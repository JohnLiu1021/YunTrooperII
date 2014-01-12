#include <stdio.h>
#include <signal.h>
#include <Urg_driver.h>
#include "../drive/drive.h"
#include "../drive/gpio.h"
#include "../MTiG/cmt3.h"

#define A 2000
#define B 1

#define EXIT_ERROR(X) {fprintf(stderr, "Error %d occured during "X": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(EXIT_FAILURE);}

int quit = 0;

BBB::GPIO led1(69, 1);
BBB::GPIO led2(68, 1);

void ctrlchandler(int sig)
{
	quit = 1;
	printf("Ctrl-C signal received.\n");
}

void exitFunc(void)
{
	(void) signal(SIGINT, SIG_DFL);
	printf("Program exit.\n");
	led1.value(0);
	led2.value(0);
}

using namespace qrk;
using namespace xsens;
using namespace YT;

pthread_mutex_t mutex;
double GlobalYaw;

void *GPSRead(void *)
{
	Cmt3 serial;
	Packet reply(1,0);
	(void) signal(SIGINT, ctrlchandler);

	if(serial.openPort("/dev/ttyUSB0", B115200) != XRV_OK)
		EXIT_ERROR("open");
	
	int timeout = 1000;
	if(serial.setTimeoutMeasurement(timeout) != XRV_OK)
		EXIT_ERROR("set timeout");
	
	CmtDeviceMode mode(CMT_OUTPUTMODE_ORIENT | 
			   CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT |
			   CMT_OUTPUTSETTINGS_ORIENTMODE_EULER |
			   CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632, 
			   20);
	if(serial.setDeviceMode(mode, false, CMT_DID_BROADCAST)) 
		EXIT_ERROR("set device mode");
	
	CmtMatrix matrix;
	matrix.m_data[0][0] = -1;
	matrix.m_data[1][1] = -1;
	matrix.m_data[2][2] = 1;
	if (serial.setObjectAlignmentMatrix(matrix) != XRV_OK) {
		EXIT_ERROR("set alignment matrix");
	}

	if(serial.gotoMeasurement())
		EXIT_ERROR("goto measurement");
	//printf("Now in measurement mode\n");

	while(!quit) {
		if (serial.waitForDataMessage(&reply) != XRV_OK) {
			EXIT_ERROR("read data message");
		}
		/*
		printf("SC: %hu ROLL:%+6.1f Pitch: %+6.1f YAW: %+6.1f     \n",
			reply.getSampleCounter(),
			reply.getOriEuler().m_roll,
			reply.getOriEuler().m_pitch,
			reply.getOriEuler().m_yaw
		      );
		*/
		pthread_mutex_lock(&mutex);
		GlobalYaw = reply.getOriEuler().m_yaw;
		pthread_mutex_unlock(&mutex);
	}
	pthread_exit(NULL);
}

int main(void)
{
	sleep(2);
	pid_t pid, sid;
	pid = fork();
	if (pid < 0) {
		perror("fork");
		exit(EXIT_FAILURE);
	}

	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	sid = setsid();
	if (sid < 0) {
		exit(EXIT_FAILURE);
	}

	(void)signal(SIGINT, ctrlchandler);
	atexit(exitFunc);
	
	YT::Drive drive;

	BBB::GPIO btn1(66, 0);
	btn1.setEdge("rising");
	led1.value(0);
	led2.value(1);

	pthread_mutex_init(&mutex, NULL);
	pthread_t thread;
	if (pthread_create(&thread, NULL, GPSRead, NULL) < 0) {
		perror("thread creation");
		exit(EXIT_FAILURE);
	}


	Urg_driver urg;
	if (!urg.open("/dev/ttyACM0")) {
		fprintf(stderr, "Error open : urg device\n");
		return -1;
	}
	urg.start_measurement(Urg_driver::Distance);

	double targetOrientation = 0.0;
	bool flag_startNav = false;
	int btnCounter = 0;
	while(!quit) {
		// Read yaw angle from other thread
		double yaw;
		pthread_mutex_lock(&mutex);
		yaw = GlobalYaw;
		pthread_mutex_unlock(&mutex);


		std::vector<long> data;
		std::vector<long> reducedData;
		std::vector<double> angleIndex;
		std::vector<long> VFH;
		std::vector<long> sectorIndex;

		long time_stamp = 0;
		if (!urg.get_distance(data, &time_stamp)) {
			perror("read : urg device");
			return -1;
		}
		
		for (int j=0, count=0; j<=672; j+=8, count+=1) {
			if (data[j] > 4000 || data[j] < 20)
				data[j] = 4000;
			reducedData.push_back(data[j]);
			angleIndex.push_back(-117.936 + (count * 2.808));
			VFH.push_back(A - (B * data[j]));
		}
		
		int sectorStart;
		int spaceCounter = 0;
		bool flag_found = false;
		int B_th = 0;
		int space_th = 8;

		// Construct Vertor Field Histogram
		for (size_t j=0; j<VFH.size(); j++) {
			if (VFH[j] <= B_th) {
				if (!flag_found) {
					flag_found = true;
					sectorStart = j;
				}
				spaceCounter++;
			} else {
				if (flag_found) {
					if (spaceCounter >= space_th) {
						sectorIndex.push_back(sectorStart);
						sectorIndex.push_back(j-1);
					}
					flag_found = false;
					spaceCounter = 0;
				}
			}
		}

		// Detect Button
		if (btn1.value()) {
			{
				if (!flag_startNav) {
					targetOrientation = yaw;
					flag_startNav = true;
				} else {
					targetOrientation = 0.0;
					flag_startNav = false;
				}
			}
		}

		// Navigation
		if (flag_startNav) {
			led1.value(1);
			double target_local = targetOrientation - yaw;
			double commandAngle = 0.0;
			
			std::vector<double> sectorAngle;
			sectorAngle.clear();
			for (size_t j=0; j<sectorIndex.size();j+=2) {
				int startIndex = sectorIndex[j];
				int endIndex = sectorIndex[j+1];
				double angle = (angleIndex[startIndex] + angleIndex[endIndex]) / 2;
				sectorAngle.push_back(angle);
				
				//printf("%d:%f to %d:%f\n", startIndex, angleIndex[startIndex], endIndex, angleIndex[endIndex]);
				//printf("average angle = %f\n", angle);
			}
			
			double min = 129600.0; // 360 * 360
			for (std::vector<double>::iterator it=sectorAngle.begin(); it!=sectorAngle.end(); it++) {
				double diff = (target_local - *it);
				if ((diff*diff) < min) {
					min = diff * diff;
					commandAngle = *it;
				}
			}
			//printf("target angle  = %f\n", commandAngle);
			drive.setSteer(commandAngle);

			if (sectorAngle.size() == 0) {
				drive.setSpeed(0);
			}else {
				drive.setSpeed(50);
			}


		} else {
			led1.value(0);
			drive.setSpeed(0);
			drive.setSteer(0);
		}
			
	}
	pthread_join(thread, NULL);

	led2.value(0);
	led1.value(0);
	printf("Reach the end of main func.\n");
	return 0;
}

