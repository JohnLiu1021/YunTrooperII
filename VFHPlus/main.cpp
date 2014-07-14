#include <stdio.h>
#include <signal.h>
#include "drive/drive.h"
#include "drive/gpio.h"
#include "MTiG/cmt3.h"
#include "vfhplus.h"
#include "Configuration/configuration.h"
#include "logfile/logfile.h"

#include <cmath>

#define MTIG_FREQ 20

#define EXIT_ERROR(X) {VFHlog.write("Error %d occured during "X": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(EXIT_FAILURE);}

#define DEBUG 1

int quit = 0;

BBB::GPIO led1(69, 1);
BBB::GPIO led2(68, 1);

LogFile VFHlog;

void ctrlchandler(int sig)
{
	quit = 1;
	VFHlog.write("Ctrl-C signal received.\n");
}

void exitFunc(void)
{
	(void) signal(SIGINT, SIG_DFL);
	VFHlog.write("Program exit.\n");
	led1.value(0);
	led2.value(0);
}

using namespace qrk;
using namespace xsens;
using namespace YT;

pthread_mutex_t mutex;
double GlobalYaw;

void *GPSRead(void *p)
{
	Cmt3 serial;
	Packet reply(1,0);
	(void) signal(SIGINT, ctrlchandler);

	if (serial.openPort("/dev/ttyUSB0", B115200) != XRV_OK)
		EXIT_ERROR("open");
	
	int timeout = 500;
	if (serial.setTimeoutMeasurement(timeout) != XRV_OK)
		EXIT_ERROR("set timeout");
	
	CmtDeviceMode2 mode(CMT_OUTPUTMODE_STATUS |
			   CMT_OUTPUTMODE_ORIENT,

			   CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT |
			   CMT_OUTPUTSETTINGS_ORIENTMODE_EULER |
			   CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632);

	mode.setSampleFrequency(MTIG_FREQ);

	if (serial.setDeviceMode2(mode, false, CMT_DID_BROADCAST))
		EXIT_ERROR("set device mode");
	
	CmtMatrix matrix;
	matrix.m_data[0][0] = -1;
	matrix.m_data[1][1] = -1;
	matrix.m_data[2][2] = 1;
	if (serial.setObjectAlignmentMatrix(matrix) != XRV_OK) {
		EXIT_ERROR("set alignment matrix");
	}

	if (serial.gotoMeasurement())
		EXIT_ERROR("goto measurement");
	//VFHlog.write("Now in measurement mode\n");

	while(!quit) {
		if (serial.waitForDataMessage(&reply) != XRV_OK) {
			EXIT_ERROR("read data message");
		}
		/*
		VFHlog.write("SC: %hu ROLL:%+6.1f Pitch: %+6.1f YAW: %+6.1f     \n",
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

class VFHConfig : public Configuration
{
public:
	VFHConfig()
	{
		struct Option optionTemp;
		
		optionTemp.name = "MaxSpeed";
		optionTemp.value = 75;
		this->options.push_back(optionTemp);

		optionTemp.name = "MinSpeed";
		optionTemp.value = 40;
		this->options.push_back(optionTemp);

		optionTemp.name = "SteeringGain";
		optionTemp.value = 4;
		this->options.push_back(optionTemp);

		optionTemp.name = "HighThreshold";
		optionTemp.value = 200;
		this->options.push_back(optionTemp);

		optionTemp.name = "LowThreshold";
		optionTemp.value = 0;
		this->options.push_back(optionTemp);

		optionTemp.name = "AngleThreshold";
		optionTemp.value = 30.0;
		this->options.push_back(optionTemp);

		optionTemp.name = "CollisionDistance";
		optionTemp.value = 450.0;
		this->options.push_back(optionTemp);

		optionTemp.name = "A";
		optionTemp.value = 1500;
		this->options.push_back(optionTemp);

		optionTemp.name = "B";
		optionTemp.value = 1;
		this->options.push_back(optionTemp);

		optionTemp.name = "BodyWidth";
		optionTemp.value = 520;
		this->options.push_back(optionTemp);

		optionTemp.name = "ROC";
		optionTemp.value = 575;
		this->options.push_back(optionTemp);

		optionTemp.name = "u1";
		optionTemp.value = 0.5;
		this->options.push_back(optionTemp);

		optionTemp.name = "u2";
		optionTemp.value = 0.2;
		this->options.push_back(optionTemp);

		optionTemp.name = "u3";
		optionTemp.value = 0.3;
		this->options.push_back(optionTemp);

		optionTemp.name = "DensityRange";
		optionTemp.value = 45;
		this->options.push_back(optionTemp);

		optionTemp.name = "LaserLog";
		optionTemp.value = 0;
		this->options.push_back(optionTemp);
	}
};

int main(void)
{
	if (VFHlog.open("/root/VFHlog"))
		printf("Erro!\n");

	int laserLogEnabled = 0;
	LogFile laserLog;
	laserLog.timeStampOFF();
	laserLog.counterStampOFF();
	laserLog.setDir("/root/LaserData");

	(void)signal(SIGINT, ctrlchandler);
	atexit(exitFunc);
	
	YT::Drive drive;

	BBB::GPIO btn2(67, 0);
	btn2.setEdge("rising");
	led1.value(0);
	led2.value(1);

	pthread_mutex_init(&mutex, NULL);
	pthread_t thread;
	if (pthread_create(&thread, NULL, GPSRead, NULL) < 0) {
		perror("thread creation");
		exit(EXIT_FAILURE);
	}

	VFHConfig config;
	double MaxSpeed = 70;
	double MinSpeed = 40;
	double SteeringGain = 3.5;

	VFHPlus vfh;
	if (!vfh.open("/dev/ttyACM0")) {
		VFHlog.write("Open LiDAR Error!\n");
		quit = 1;
	} else {
		vfh.setScanningParameter(-120, 120, 4);
        
		if (config.readFromFile("VFH.conf") < 0) {
			VFHlog.write("Config file does not exist, using default config and create it.\n");
			config.writeToFile("VFH.conf");
		}
        
		double value1, value2, value3;
        
		if (config.search("MaxSpeed", value1))
			MaxSpeed = value1;
        
		if (config.search("MinSpeed", value1))
			MinSpeed = value1;
        
		if (config.search("SteeringGain", value1))
			SteeringGain = value1;
        
		if (config.search("ROC", value1)) {
			vfh.setRadiusOfCurvature(value1);
			VFHlog.write("ROC: %f\n", vfh.getRadiusOfCurvature());
		}
        
		if (config.search("BodyWidth", value1)) {
			vfh.setBodyWidth(value1);
			VFHlog.write("BodyWidth: %d\n", vfh.getBodyWidth());
		}
        
		if (config.search("DensityRange", value1)) {
			vfh.setDensityRange(value1);
			VFHlog.write("DensityRange: %f\n", vfh.getDensityRange());
		}
        
		if (config.search("AngleThreshold", value1)) {
			vfh.setAngleThreshold(value1);
			VFHlog.write("AngleThreshold: %f\n", vfh.getAngleThreshold());
		}

		if (config.search("CollisionDistance", value1)) {
			vfh.setCollisionDistance(value1);
			value1 = vfh.getCollisionDistance();
			VFHlog.write("Collision Distance: %f\n", value1); 
		}
        
		if (config.search("A", value1) &&
		    config.search("B", value2) ) {
			vfh.setVFHPlusParameter(value1, value2);
			vfh.getVFHPlusParameter(value1, value2);
			VFHlog.write("VFH Para.: %f, %f\n", value1, value2);
		}
        
		if (config.search("LowThreshold", value1) &&
		    config.search("HighThreshold", value2)) {
			vfh.setVFHThreshold(value1, value2);
			VFHlog.write("VFH Threshold: low:%f, high: %f\n", value1, value2);
		}
		
		if (config.search("u1", value1) &&
		    config.search("u2", value2) && 
		    config.search("u3", value3)) {
			vfh.setCostFuncParameter(value1, value2, value3);
			vfh.getCostFuncParameter(value1, value2, value3);
			VFHlog.write("u1: %f, u2: %f, u3: %f\n", value1, value2, value3);
		}
        
		if (config.search("LaserLog", value1)) {
			laserLogEnabled = (int)value1;
			VFHlog.write("Laser Log Enabled: %d\n", laserLogEnabled); 
		}
		vfh.start();
	}

	double targetOrientation = 0.0;
	bool flag_startNav = false;
	

	int previousBtnStatus = 0;
	int presentBtnStatus = 0;

	double target_local = 0.0;
	double commandAngle = 0.0;
	int speedValue = 0;
	int steerValue = 0;

	VFHlog.flush();

	while(!quit) {
		// Update VFH algorithm.
		vfh.update();

		// Read yaw angle from other thread
		double yaw;
		pthread_mutex_lock(&mutex);
		yaw = GlobalYaw;
		pthread_mutex_unlock(&mutex);

		// Detect Button
		presentBtnStatus = btn2.value();

		if (presentBtnStatus != previousBtnStatus) {
			if (presentBtnStatus) {
				targetOrientation = yaw;
				flag_startNav = true;

				if (laserLogEnabled) {
					laserLog.open();
					std::vector<double> angle;
					std::vector<double>::iterator it;
					vfh.getCorrespondAngle(angle);
					/* 
					 * Configuration
					 * 1. Laser data length
					 * 2. Robot dimension
					 * 3. Density range
					 * 4. Tau_angle
					*/
					laserLog.write("%d %d %f %f\n",
						angle.size(),
						vfh.getBodyWidth(),
						vfh.getDensityRange(),
						vfh.getAngleThreshold());
                                
					for (it=angle.begin(); it!=angle.end(); it++)
						laserLog.write("%3.4f ", *it);
					laserLog.write("\n");
				}
			} else {
				targetOrientation = 0.0;
				flag_startNav = false;
				if (laserLogEnabled)
					laserLog.close();
			}
		}

		// Navigation
		if (flag_startNav) {
			target_local = targetOrientation - yaw;
			if (target_local > 180)
				target_local -= 360;
			else if (target_local < -180) 
				target_local += 360;

			commandAngle = vfh.calculateDirection(target_local);

			if (commandAngle <= 180) {
				double density = vfh.getDensity();
				printf("density = %f\n", density);
				speedValue = (int)((MaxSpeed - MinSpeed) * (1.0 - density) + MinSpeed);
				steerValue = (int)(commandAngle * SteeringGain);
			} else {
				speedValue = (int)MinSpeed;
			}

			if (vfh.collisionDetected(((double)steerValue) / SteeringGain))
				speedValue = 0;

			drive.setSpeed(speedValue);
			drive.setSteer(steerValue);

			// Status indicator
			if (commandAngle > 180)
				led1.toggle();
			else
				led1.value(1);

			if (speedValue == 0)
				led2.toggle();
			else
				led2.value(1);

			if (laserLogEnabled) {
				std::vector<long> data;
				std::vector<long>::iterator it;
				vfh.getMeasuredDistance(data);
				for (it=data.begin(); it!=data.end(); it++)
					laserLog.write("%4d ", *it);
				laserLog.write("%3.2f %3.2f %d %d\n", target_local, commandAngle, steerValue, speedValue);
			}
		} else {
			led1.value(0);
			led2.value(1);
			drive.setSpeed(0);
			drive.setSteer(0);
		}

		previousBtnStatus = presentBtnStatus;
			
	}
	printf("Ctrl-C received!\n");
	pthread_join(thread, NULL);

	led2.value(0);
	led1.value(0);
	VFHlog.write("Reach the end of main func.\n");
	exit(EXIT_FAILURE);
}

