#include <px4.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <math.h>
#include <stdint.h>
#include <sys/select.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>	//AccZ(vibration) and angle oscillation


#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <limits.h>
#include <nuttx/serial/serial.h>
#include <time.h>


#define MSG_LENGTH 11

//#define DEBUG_PRINTF //comment this line to stop printf

extern "C" __EXPORT int to_omv_main(int argc, char *argv[]);
static void usage(void);

class toOmv {
public:
    /**
     * Constructor
     */
    toOmv();

    /**
     * Destructor, also kills task.
     */
    ~toOmv();

    /**
     * Start task.
     *
     * @return      OK on success.
     */
    int start();

private:

    bool _task_should_exit; /**< if true, task should exit */
	int _control_task; /**< task handle for task */
	
	unsigned char msgToSend[MSG_LENGTH];
	int roll,pitch;

	//System Variables
	int uart;

	//Topic Subscription
	int sensor_combined_sub_fd;

	struct sensor_combined_s sensor;

	//GCS - Leader Communication Network Message Lengths
	int MSGLENGTH_COMMAND; //should be 10
	
    //Shim for calling task_main from task_create.
    static void task_main_trampoline(int argc, char *argv[]);

    //Main task.
    void task_main();
	
    //Check for changes in subscribed topics.
	void poll_subscriptions();

	//Set up UART port for receiving messages
	bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

	//Checksum helper function
	uint16_t crc16_update(uint16_t crc, char a);
	
	//Checksum calculation
	uint16_t calcCRC16(unsigned char *serno, int length);

	//Checksum calculation
	uint16_t checksum(unsigned char *data, int length);

	//Initialize UART
	void uart_init();
	
	//Create 2 byte array from integer
	void getTwoByteArray(int num, unsigned char *array);

	//Create 4 byte array from integer
	void getFourByteArray(int num, unsigned char *array);

	//Copy array into another
	void copyArray(unsigned char *dest, int destIndex, unsigned char *source, int sourceLength);

	//Send all vehicle status in the group
	void send_all_position(int groupID, int ownID);
	//void send_all_position(int groupID, int ownID);

	//Compute vehicle status using 3 parameters: gps status, arm status, and flight mode
	int vehicleStatus(bool gps, bool armed, bool automode);

	void send_rp_msg();

	//Create position packet for individual vehicle
	void createVehiclePosPacket();
};

namespace to_omv {

toOmv *to_omv;

}

toOmv::toOmv() :
		_task_should_exit(false),
		_control_task(-1),
		uart(-1),
        sensor_combined_sub_fd(-1),
		MSGLENGTH_COMMAND(10)
{
	memset(&sensor, 0, sizeof(sensor));
}

toOmv::~toOmv()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    to_omv::to_omv = nullptr;
}

static void usage()
{
	fprintf(stderr,
			"usage: to_omv start [-d <devicename>]\n"
			"       to_omv stop\n"
			"       to_omv status\n");
	exit(1);
}

int to_omv_main(int argc, char *argv[]) {
    if (argc < 1) {
        errx(1, "usage: to_omv {start|stop|status}");
    }

    if (!strcmp(argv[1], "start")) {

        if (to_omv::to_omv != nullptr) {
            errx(1, "already running");
        }

        to_omv::to_omv = new toOmv;

        if (to_omv::to_omv == nullptr) {
            errx(1, "alloc failed");
        }

        if (OK != to_omv::to_omv->start()) {
            delete to_omv::to_omv;
            to_omv::to_omv = nullptr;
            err(1, "start failed");
        }

        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (to_omv::to_omv == nullptr) {
            errx(1, "not running");
        }

        delete to_omv::to_omv;
        to_omv::to_omv = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (to_omv::to_omv) {
            errx(0, "running");

        } else {
            errx(1, "not running");
        }
    }

    warnx("unrecognized command");

    usage();
    return 1;
}

int toOmv::start() {
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("[ATL] to_omv",     //name
            SCHED_DEFAULT,                          //priority
            SCHED_PRIORITY_DEFAULT,                 //scheduler
            1500,                                   //stack size
            (main_t) &toOmv::task_main_trampoline, nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void toOmv::task_main_trampoline(int argc, char *argv[])
{
    to_omv::to_omv->task_main();
}


void toOmv::task_main()
{
	warnx("[Ground Comm] starting\n");



	//Subscribing Topics
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	//Initialize UART Port
	uart_init();

	while (!_task_should_exit) {
		poll_subscriptions();
		createVehiclePosPacket();
		send_rp_msg();
		

		usleep(30000); //30ms
	}

    _control_task = -1;
    _exit(0);
}

bool toOmv::setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		//fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);
	config.c_oflag = 0;
	config.c_lflag = 0;
	config.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	config.c_cflag |= CS8;
	config.c_cflag &= ~CRTSCTS;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 5; // was 0
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			//fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}
	if(tcsetattr(fd, TCSANOW, &config) < 0)
	{
		//fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

uint16_t toOmv::crc16_update(uint16_t crc, char a)
{
	int i;
	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
		if ((crc & 1) != 0)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

uint16_t toOmv::calcCRC16(unsigned char *serno, int length)
{
		uint16_t crc = 0;
	uint16_t i;

	unsigned char temp[31] = {0};
	memcpy(temp, &serno[0], length);

	for (i = 0; i < 31; i++)
		crc = crc16_update(crc, temp[i]);

	return crc; // must be 0
}

uint16_t toOmv::checksum(unsigned char *data, int length)
{
	uint16_t sum = calcCRC16(data, length);
	return sum;
}

void toOmv::uart_init()
{
	uart  = open("/dev/ttyS1", O_RDWR | O_NOCTTY);		// ttyS1 = telem1 port on pixhawk
	setup_port(uart,57600,8,1,false,false); 			//baudrate = 57600
}

void toOmv::poll_subscriptions()
{
	bool updated;

	orb_check(sensor_combined_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor);
		roll = int (sensor.accelerometer_m_s2[1]*10000);
		pitch =int (sensor.accelerometer_m_s2[0]*10000); //This may not be correct way to call the updated msg?
		//	printf("roll, pitch float: %.2f \t\t %.2f \n", roll,pitch);	
	//			printf("roll, pitch int: %d \t\t %d \n", ((int) roll),((int) pitch));	
	//	printf("In callback: roll, pitch: %f \t\t %f", roll,pitch);	
	}
}


void toOmv::getTwoByteArray(int num, unsigned char *array)
{
	if(num<(-32768) || num>32767)
	{
		printf("Error getTwoByteArray: trying to fill a 2 byte int into too small an array");
	} 
	//Masks out each Byte of number and assigns to character array
	array[1] = (num >> 8) & 0xFF;
	array[0] = num & 0xFF;
}


/*Converts integer to 4 Byte character array*/
void toOmv::getFourByteArray(int num, unsigned char *array)
{
	//Masks out each Byte of number and assigns to character array
	array[3] = (num >> 24) & 0xFF;
	array[2] = (num >> 16) & 0xFF;
	array[1] = (num >> 8) & 0xFF;
	array[0] = num & 0xFF;
}

/*Copies array source into array dest, beginning at location destIndex
 * destIndex+sourceLength must be less than length of dest or else out of bounds
 */
void toOmv::copyArray(unsigned char *dest, int destIndex, unsigned char *source, int sourceLength)
{
	int i;
	for (i = 0; i < sourceLength; i++)
	{
		dest[destIndex+i] = source[i];
	}
}	


void toOmv::send_rp_msg()
{
	write(uart,msgToSend,MSG_LENGTH);
	usleep(1000);
}

void toOmv::createVehiclePosPacket()
{
	//Packet header, not using for now
	char header[] = "X";
	unsigned char roll_ch[4],pitch_ch[4], checkSum_char[2];
	//getFourByteArray(roll,roll_ch);
	//getFourByteArray(pitch,pitch_ch);
getFourByteArray(4294967295,roll_ch);
getFourByteArray(4294967295,pitch_ch);
	
	
	msgToSend[0] = header[0];
	copyArray(msgToSend, 1, roll_ch, 4);
	copyArray(msgToSend, 5, pitch_ch, 4);	

	unsigned char toCheck[8];
	memcpy(toCheck, &msgToSend[1],8);

	printf("roll, pitch, 8 byte int: %d \t %d \t %x %x %x %x \t  %x %x %x %x\n",roll,pitch, roll_ch[3],roll_ch[2],roll_ch[1],roll_ch[0], pitch_ch[3],pitch_ch[2],pitch_ch[1],pitch_ch[0]);

	int checkSum = checksum(toCheck, 8);
	getFourByteArray(checkSum, checkSum_char);
	copyArray(msgToSend, 9, checkSum_char, 2);
}
