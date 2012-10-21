/* -------------------------------------------------------------------------- */
// BMA180 Spi Communication Node (Mux+Counter)
//
//    - Reads from 8 BMA 180's with specified kHz Sampling frequency
//    - Writes message to shared memory with accel data
//
//  created by: Sebastian Haug 2010 | Robert Bosch RTC
//  edited by : Nikhil Deshpande 2011 | Robert Bosch RTC
/* -------------------------------------------------------------------------- */

// Includes
/* -------------------------------------------------------------------------- */
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <ctime>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <linux/spi/spidev.h>
#include <sched.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gumstix_memwrite_bma180/writermsg.h>
#include <errno.h>

using namespace std;


// Macros, Defines
/* -------------------------------------------------------------------------- */
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define NUM_SENSORS 8 
#define SHMSZ	30720000	
#define xSHMSZ	0x1D4C000  // hex converted

// Configuration
/* -------------------------------------------------------------------------- */
static const char *device0 	= "/dev/spidev1.1";
static uint8_t mode 		= SPI_MODE_3;
static uint8_t bits 		= 8;
static uint32_t speed 		= 6000000;
static uint16_t delay 		= 0; //in us


// Static helper functions
/* -------------------------------------------------------------------------- */
static void pabort(const char *s) {
	perror(s);
	abort();
}

static void conv_device(int ret, int fd) {
	// Spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	// Bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	// Max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d MHz)\n", speed, speed / 1000000);
}

int16_t reint_as_2c(char chMSB, char chLSB)
{
	// Reinterprets unsigned integer as 7 bit two's complement number and returns signed integer
	int16_t 	s16int;			//2 byte int to build message
	//verify if pos or neg
	if ( (chMSB & 0x80) == 0 ) {
        	//positive number
		s16int = (int16_t) ((((uint16_t)chMSB)&0xFF)<<6) + ((((uint16_t)chLSB)&0xFC)>>2);
	}
	else {
        	//negative number
	        //set MSB (sign) to zero and build 2 complement unsigned; offset 8192 for 2^13
        	s16int = (int16_t) (((((uint16_t)chMSB)&0x7F)<<6) + ((((uint16_t)chLSB)&0xFC)>>2) - 8192);
	}

	return s16int;
}

int udelay (unsigned int usecs) {
	struct timeval StartTime, EndTime, CurrentTime, delay_r;
	delay_r.tv_sec = 0;
	delay_r.tv_usec = usecs;
	gettimeofday(&StartTime, NULL);
	timeradd(&StartTime, &delay_r, &EndTime);
	//cout << "usecs: " << usecs << endl;
	//if usecs > 10000, then usleep works well using fewer resources
	if (usecs >= 10000) {
		usleep(usecs);
		return 1;
	}
	// if usecs < 10000, use tighter loop...
	do {
		gettimeofday(&CurrentTime, NULL);
	} while (timercmp(&CurrentTime, &EndTime, <));
	return 1;
}

// Main
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
	struct sched_param prmtr;
	prmtr.sched_priority = 99;
	sched_setscheduler(0,SCHED_FIFO,&prmtr);

	// Ros
	ros::init(argc, argv, "gumstix_memwrite_bma180");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<gumstix_memwrite_bma180::writermsg> ("writermsg", 10);	
	gumstix_memwrite_bma180::writermsg msg;

	double rate_Hz;
	int duration;
	// Initialize members
	int ret = 0, fd0;

	// Open SPI device 0
	fd0 = open(device0, O_RDWR);
	if (fd0 < 0)
		pabort("can't open device");
	conv_device(ret, fd0);

	// get Parameters
	if (n.getParam("/gumstix_memwrite_bma180/rate_Hz", rate_Hz)==false) {
		rate_Hz = 1200;
	}
	if (n.getParam("/gumstix_memwrite_bma180/duration", duration)==false) {
		duration = 2;
	}

	// Initialize measurement buffer
	/* -------------------------------------------------------------------------- */
	// Init tx buffer
	uint8_t tx_meas[] = {0x82, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};
	//                   addr, xlsb, xmsb, ylsb, ymsb, zlsb, zmsb

	// Init rx buffer
	char rx_meas[ARRAY_SIZE(tx_meas)] = {0};
	int16_t valAvail[NUM_SENSORS][3];
	int16_t valAccel[NUM_SENSORS][3];
	long i=0, k=0, msgs_not_logged=0, msgs_undersampled=0;
	uint64_t j=0;
	long int num_msgs_sampled=1;

	// Prepare spi message for acceleration measurement
	struct spi_ioc_transfer tr_acc;
	tr_acc.tx_buf 			= (unsigned long) tx_meas;
	tr_acc.rx_buf 			= (unsigned long) rx_meas;
	tr_acc.len 			= ARRAY_SIZE(tx_meas);
	tr_acc.delay_usecs 		= delay;
	tr_acc.speed_hz 		= speed;
	tr_acc.bits_per_word 		= bits;
	tr_acc.cs_change 		= true;
	/* -------------------------------------------------------------------------- */

	// Initialize Shared Memory
	int16_t *shm, *s, shmid;
	key_t key = 42;
	if ((shmid = shmget(key, SHMSZ, IPC_CREAT | 0666)) < 0)	{
	    if(errno == EINVAL)
                printf("Invalid segment size specified\n");
            else if(errno == EEXIST)
                printf("Segment exists, cannot create it\n");
            else if(errno == EIDRM)
                printf("Segment is marked for deletion or was removed\n");
            else if(errno == ENOENT)
                printf("Segment does not exist\n");
            else if(errno == EACCES)
                printf("Permission denied\n");
            else if(errno == ENOMEM)
                printf("Not enough memory to create segment\n");

	}
	if ((shm = (int16_t *) shmat(shmid, NULL, 0)) == (int16_t *) -1) {
		cerr << "shmat did not work! " << shm << endl;
	}
	s = shm;
	s++;
	*shm = 10001;
	cout << "Begin memory <writer> : " << s << endl;
	// Rate object
	//ros::Rate loop_rate(rate_Hz);
	cout << "Sampling rate: " << rate_Hz << " Hz" << endl;
	cout << "Sampling duration: " << duration << " seconds" << endl;

	struct timeval start_t, curr_t, endLoop, loop_t, beforeProc, afterProc, delay_t;
	unsigned int delVal = ((unsigned int)(1000000/rate_Hz)); 
	long int rem_time;

	// Main loop
	// specify data logging duration in seconds...
	bool doOnce=true;
	int pubThresh=100*duration;
	loop_t.tv_sec = duration;	loop_t.tv_usec = 0;
	gettimeofday(&start_t, NULL);
	timeradd(&start_t, &loop_t, &endLoop);
	do {
		gettimeofday(&beforeProc, NULL);
		// Transfer/receive messages
		for (i = 0; i < NUM_SENSORS; i++) {
			if(s>=(shm + xSHMSZ)){
				s = shm;
				s++;
			}
			// Do Transfer
			ret = ioctl(fd0, SPI_IOC_MESSAGE(1), &tr_acc); 		// get accel

			// Extract new data bit
			valAvail[i][0] = rx_meas[1] & 0x1; 	// new x
			valAvail[i][1] = rx_meas[3] & 0x1; 	// new y
			valAvail[i][2] = rx_meas[5] & 0x1; 	// new z

			// Extract and convert acceleration values
			valAccel[i][0] = reint_as_2c(rx_meas[2], rx_meas[1]); 	// acc x
			valAccel[i][1] = reint_as_2c(rx_meas[4], rx_meas[3]); 	// acc y
			valAccel[i][2] = reint_as_2c(rx_meas[6], rx_meas[5]); 	// acc z
			for (k=0; k<3 ;k++) {
				*(s++) = valAvail[i][k];
				if(s>=(shm + xSHMSZ)){
					s = shm;
					s++;
				}
				j++;
			}
			for (k=0; k<3; k++) {
				*(s++) = valAccel[i][k]; 
				if(s>=(shm + xSHMSZ)){
					s = shm;
					s++;
				}
				j++;
			}
		}
		if(num_msgs_sampled>=pubThresh){
			msg.written = true;
			if(doOnce){
				doOnce = false;
				pub.publish(msg);
				ros::spinOnce();
			}
		} else {
			msg.written = false;
		}
		num_msgs_sampled++;
		gettimeofday(&afterProc, NULL);
		timersub(&afterProc, &beforeProc, &delay_t);
		rem_time = delVal - ((delay_t.tv_sec*1000000) + delay_t.tv_usec);
		//cout << "Process time: " << delay_t.tv_usec << " Delay: " << rem_time << endl;
		if(rem_time>0) {
			udelay(rem_time);
		} else {
			//cout << " ** Process delay longer than desired Loop rate: " << rem_time << endl;
			msgs_undersampled++;// = num_msgs_sampled;
		}
		gettimeofday(&curr_t, NULL);
		//cout << "Loop Time: " << curr_t.tv_usec << endl;
		//cout << curr_t.tv_sec << " <compare> " << endLoop.tv_sec << endl;
	} while(timercmp(&curr_t, &endLoop, <));
	// Close spi devices
	close(fd0);
	*shm = -112;
	cout << "Done writing..." << endl;
	cout << "Values stored: " << j << endl;
	cout << "Msgs sampled: " << num_msgs_sampled << endl;
//	while(*shm != (j+1)){
//		udelay(6500);
//	}
	cout << "End memory <writer> : " << s << endl; 
//	msg.written = false;
//	pub.publish(msg);
	exit(0);
}
