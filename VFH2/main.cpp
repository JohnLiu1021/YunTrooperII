#include <stdio.h>
#include <signal.h>
#include <Urg_driver.h>
#include "drive/drive.h"
#include "drive/gpio.h"
#include "MTiG/cmt3.h"
#include "vfh2.h"

#include <cmath>

#define EXIT_ERROR(X) {fprintf(stderr, "Error %d occured during "X": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(EXIT_FAILURE);}

#define ALPHA 0.3
#define BETA 0.7
#define STEERING_GAIN 1.4
#define MAX_SPEED 70 
#define MIN_SPEED 30

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
	sleep(1);
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

	VFH vfh;
	vfh.open("/dev/ttyACM0");

	vfh.setScanningParameter(-90, 90, 4);
	vfh.setBodyWidth(400);
	vfh.setDetectionZone(RECT, 500, 1500);
	vfh.setDensityThreshold(400);
	vfh.setSpaceThreshold(450);
	
	vfh.start();

	double targetOrientation = 0.0;
	bool flag_startNav = false;

	std::vector<VFH::Sector>::iterator it_s;
	while(!quit) {
		// Update VFH algorithm.
		vfh.update();

		// Read yaw angle from other thread
		double yaw;
		pthread_mutex_lock(&mutex);
		yaw = GlobalYaw;
		pthread_mutex_unlock(&mutex);

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
			double target_local = targetOrientation - yaw;

			if (vfh.collisionDetected()) {
				drive.setSpeed(0);
				led1.toggle();

			} else if (vfh.obstacleDetected()) {
				double max = 0.0;
				double commandAngle = 0.0;
				int i=0;
				for (it_s=vfh.sector.begin(); it_s!=vfh.sector.end(); it_s++) {
					i++;
					double diff = target_local - it_s->direction;
					if (diff > 180)
						diff -= 360;
					else if (diff < -180)
						diff += 360;

					diff = fabs(diff) / 180;
					
					double width = (double)it_s->width / (double)vfh.getTotalStep();

					double objval = (ALPHA * diff) +  (BETA * width);

					if (it_s == vfh.sector.begin()) {
						max = objval;
						commandAngle = it_s->direction;
					} else {
						if (objval > max) {
							max = objval;
							commandAngle = it_s->direction;
						}
					}
					printf("#%d: %d, %f, obj: %f, ", i, it_s->width, it_s->direction, objval);
				}
				printf(" : Maxobj: %f\n", max);

				drive.setSteer((int)(commandAngle * STEERING_GAIN));
				printf("Steer command  = %d, ", (int)(commandAngle * STEERING_GAIN));
				printf("density = %f\n", vfh.getDensity());
                        
				if (vfh.sector.empty()) {
					drive.setSpeed(0);
					led1.toggle();
				} else {
					double speedValue = (1.0 - vfh.getDensity()) * MAX_SPEED;
					if (speedValue < MIN_SPEED)
						speedValue = MIN_SPEED;

					drive.setSpeed((int)speedValue);
					led1.value(1);
				}
			} else {
				int commandAngle = (int)(target_local * STEERING_GAIN);
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

