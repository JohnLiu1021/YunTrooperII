#include <stdio.h>
#include <signal.h>
#include <Urg_driver.h>
#include "drive/drive.h"
#include "drive/gpio.h"
#include "MTiG/cmt3.h"
#include "vfh.h"

#define EXIT_ERROR(X) {fprintf(stderr, "Error %d occured during "X": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(EXIT_FAILURE);}

#define ALPHA 0.3
#define BETA 0.7
#define SKIP_INDEX 4
#define BODY_WIDTH 200
#define STEERING_GAIN 1.5
#define MAX_SPEED 75 

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
	/*
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
	*/
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
	int min_step = urg.min_step();
	int max_step = urg.max_step();

	std::vector<double> angle;
	for (int i=min_step; i<=max_step; i+=SKIP_INDEX) {
		angle.push_back(urg.step2deg(i));
	}

	VFH vfh;
	vfh.setAngleIndex(angle);
	vfh.setDetectionZone(60.0, 2000);

	double targetOrientation = 0.0;
	bool flag_startNav = false;
	while(!quit) {
		// Read yaw angle from other thread
		double yaw;
		pthread_mutex_lock(&mutex);
		yaw = GlobalYaw;
		pthread_mutex_unlock(&mutex);

		std::vector<long> data;
		std::vector<long> reducedData;

		long time_stamp = 0;
		if (!urg.get_distance(data, &time_stamp)) {
			perror("read : urg device");
			return -1;
		}
		
		// Reducing the number of data
		for (int j=urg.step2index(min_step); j<urg.step2index(max_step); j+=SKIP_INDEX) {
			if (data[j] > 4000 || data[j] < 20)
				data[j] = 4000;
			reducedData.push_back(data[j] - BODY_WIDTH);
		}

		// Update
		vfh.update(reducedData);
		
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

			if (vfh.obstacleDetected()) {
				double max = 0.0;
				double commandAngle = 0.0;
				for (size_t i=0; i<vfh.sector.size(); i++) {
					double diff = (target_local - vfh.sector[i].angle);
					if (diff < 0) 
						diff = -diff;
					diff = 1.0 - (diff / 180.0);
					
					int totalStep = vfh.getTotalStep();
					double width = (double)vfh.sector[i].width / (double)totalStep;

					double objval = (ALPHA * diff) +  (BETA * width);

					if (i == 0) {
						max = objval;
						commandAngle = vfh.sector[i].angle;
					} else {
						if (objval > max) {
							max = objval;
							commandAngle = vfh.sector[i].angle;
						}
					}
				}

				drive.setSteer((int)(commandAngle * STEERING_GAIN));
				printf("target angle  = %d\n", (int)(commandAngle * STEERING_GAIN));
				printf("density = %f\n", vfh.getDensity());
                        
				if (vfh.sector.size() == 0) {
					drive.setSpeed(0);
				} else {
					double speedValue = (1.0 - vfh.getDensity()) * MAX_SPEED;
					drive.setSpeed((int)speedValue);
					printf("speedValue = %d\n", (int)speedValue);
				}
			} else {
				int commandAngle = (int)(target_local);
				printf("commandAngle = %d\n", commandAngle);
				drive.setSteer(commandAngle);
				drive.setSpeed(MAX_SPEED);
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

