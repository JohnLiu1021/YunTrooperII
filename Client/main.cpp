#include "communicator/communicator.h"
#include "communicator/packetclient.h"
#include "PathPoints/pathpoints.h"
#include "drive/drive.h"
#include "drive/gpio.h"
#include "MTiG/cmt3.h"
#include "VFHPlus/vfhplus.h"
#include "logfile/logfile.h"
#include "Configuration/configuration.h"

#include <errno.h>
#include <atomic>

#include <GeographicLib/Geodesic.hpp>

#define MTIG_FREQ 20
//#define DEBUG 1

/* Atomic quit bit */
std::atomic_char quit(0);

/* Global data logger */
LogFile syslog;

using namespace YT;
using namespace xsens;
using namespace qrk;
using namespace GeographicLib;

struct MTiGData {
	pthread_mutex_t mutex;
	unsigned char status;
	double latitude;
	double longitude;
	double yaw;
	CmtGpsStatus gpsStatus;
};

class Config : public Configuration
{
public:
	Config()
	{
		struct Option optionTemp;
	
		optionTemp.name = "PositionError";
		optionTemp.value = 1;
		this->options.push_back(optionTemp);

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

	}
};

/* Create a thread to read from MTiG, which implemented the asynchronous read */
void *GPSRead(void *ptr)
{
	struct MTiGData *data = (struct MTiGData*)ptr;

	Cmt3 serial;
	xsens::Packet reply(1,0);

	if (serial.openPort("/dev/ttyUSB0", B115200) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during open MTiG: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
		pthread_exit(NULL);
	}
	
	int timeout = 1000;
	if (serial.setTimeoutMeasurement(timeout) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set timeout: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}

	/* Set MTiG scenario
	 * 1: General
	 * 2: Automotive
	 */
	if (serial.setScenario(2) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set scenario: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}
	
	CmtDeviceMode2 mode(CMT_OUTPUTMODE_STATUS |
			   CMT_OUTPUTMODE_ORIENT |
			   CMT_OUTPUTMODE_POSITION,

			   CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT |
			   CMT_OUTPUTSETTINGS_ORIENTMODE_EULER |
			   CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632);

	mode.setSampleFrequency(MTIG_FREQ);

	if (serial.setDeviceMode2(mode, false, CMT_DID_BROADCAST)) {
		syslog.write("Error: MTiG: %d occured during set device mode: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit = true;
	}

	CmtMatrix matrix;
	matrix.m_data[0][0] = -1;
	matrix.m_data[1][1] = -1;
	matrix.m_data[2][2] = 1;
	if (serial.setObjectAlignmentMatrix(matrix) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set alignment matrix: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}
	
	CmtVector v;
	/* GPS lever arm vector on object coor. */
	v.m_data[0] = -0.197;
	v.m_data[1] = 0.057;
	v.m_data[2] = 0.14;

	if (serial.setGpsLeverArm(v) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set GPS lever arm: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}

	if (serial.gotoMeasurement()) {
		syslog.write("Error: MTiG: %d occured during go to measurment: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}

	XsensResultValue XRet;
	int MTiGErrorCounter = 0;

	while (!quit.load(std::memory_order_relaxed)) {
		XRet = serial.waitForDataMessage(&reply);
		if (XRet != XRV_OK) {
			if (XRet == XRV_TIMEOUT) {
				MTiGErrorCounter++;
				syslog.write("Warning: MTiG: Timeout occurred, %d times\n", MTiGErrorCounter);
				if (MTiGErrorCounter >= 10) {
					syslog.write("Error: MTiG: Too many error.\n");
					quit.store(1, std::memory_order_relaxed);
				}
			} else {
				syslog.write("Error: MTiG: %d occured during read data message: %s\n",
					serial.getLastResult(),
					xsensResultText(serial.getLastResult()));
				quit.store(1, std::memory_order_relaxed);
			}
		}

		CmtGpsStatus gpsStatus;
		XRet = serial.getGpsStatus(gpsStatus);
		if (XRet != XRV_OK) {
			if (XRet == XRV_TIMEOUT) {
				MTiGErrorCounter++;
				syslog.write("Warning: MTiG: Timeout occurred, %d times\n", MTiGErrorCounter);
				if (MTiGErrorCounter >= 10) {
					syslog.write("Error: MTiG: Too many error.\n");
					quit.store(1, std::memory_order_relaxed);
				}
			} else {
				syslog.write("Error: MTiG: %d occured during read GPS status: %s\n",
					serial.getLastResult(),
					xsensResultText(serial.getLastResult()));
				quit.store(1, std::memory_order_relaxed);
			}
		}

		MTiGErrorCounter = 0;
		pthread_mutex_lock(&(data->mutex));
		data->status = reply.getStatus();
		data->yaw = reply.getOriEuler().m_yaw;
		data->latitude = reply.getPositionLLA().m_data[0];
		data->longitude = reply.getPositionLLA().m_data[1];
		data->gpsStatus = gpsStatus;
		pthread_mutex_unlock(&(data->mutex));
	}

	pthread_exit(NULL);
}

int main(void)
{
	/* Setting up Navigation Log */
	LogFile navlog;
	navlog.timeStampOFF();
	navlog.counterStampOFF();
	navlog.setDir("/root/NavLog");

	/* Setting up Log file */
	pthread_mutex_t syslogMutex;
	pthread_mutex_init(&syslogMutex, NULL);
	syslog.setMutex(&syslogMutex);
	syslog.open("/root/syslog");

	/* Setting up communicator */
	Communicator communicator;
	communicator.open("/dev/ttyO4", 38400);
	PacketClient packet;

	/* Setting up motion driver */
	Drive drive;
	struct ControlParameter speedPara = drive.getSpeedParameter();
	struct ControlParameter steerPara = drive.getSteerParameter();

	// Setting deadzone
	speedPara.deadzone = 10;

	// Setting the range of speed parameter
	speedPara.range = 320000;

	steerPara.deadzone = 10;
	drive.setSpeedParameter(speedPara);
	drive.setSteerParameter(steerPara);

	/* Setting up leds and buttons */
	BBB::GPIO led1(69, 1);
	BBB::GPIO led2(68, 1);
	led1.value(0);
	led2.value(0);
	BBB::GPIO btn1(66, 0);
	BBB::GPIO btn2(67, 0);

	/* Creating GPS reading thread */
	struct MTiGData MData;
	pthread_mutex_init(&(MData.mutex), NULL);
	pthread_t thread;
	if (pthread_create(&thread, NULL, GPSRead, (void*)&MData) < 0) {
		syslog.write("Error: Main: Thread Creation failed : %s\n", strerror(errno));
		quit.store(1, std::memory_order_relaxed);
	}

	/* Configuration class and variable */
	Config config;
	double PositionError = 1;
	double MaxSpeed = 80;
	double MinSpeed = 55;
	double SteeringGain = 3.5;
	double YawOffset = 0.0;
        
	/* VFH, LiDAR and configuration. */
	VFHPlus vfh;
	if (!vfh.open("/dev/ttyACM0")) {
		syslog.write("Error: Main: Unable to open LiDAR: %s\n", vfh.what());
		quit.store(1, std::memory_order_relaxed);
	} else {
	;	vfh.setScanningParameter(-120, 120, 4);
        
		if (config.readFromFile("Navigation.conf") < 0) {
			syslog.write("Config file does not exist, using default config and create it.\n");
			config.writeToFile("Navigation.conf");
		}
        
		double value1, value2, value3;
        
		if (config.search("PositionError", value1))
			PositionError = value1;

		if (config.search("MaxSpeed", value1))
			MaxSpeed = value1;
        
		if (config.search("MinSpeed", value1))
			MinSpeed = value1;
        
		if (config.search("SteeringGain", value1))
			SteeringGain = value1;
        
		if (config.search("ROC", value1)) {
			vfh.setRadiusOfCurvature(value1);
			syslog.write("ROC: %f\n", vfh.getRadiusOfCurvature());
		}
        
		if (config.search("BodyWidth", value1)) {
			vfh.setBodyWidth(value1);
			syslog.write("BodyWidth: %d\n", vfh.getBodyWidth());
		}
        
		if (config.search("DensityRange", value1)) {
			vfh.setDensityRange(value1);
			syslog.write("DensityRange: %f\n", vfh.getDensityRange());
		}
        
		if (config.search("AngleThreshold", value1)) {
			vfh.setAngleThreshold(value1);
			syslog.write("AngleThreshold: %f\n", vfh.getAngleThreshold());
		}

		if (config.search("CollisionDistance", value1)) {
			vfh.setCollisionDistance(value1);
			value1 = vfh.getCollisionDistance();
			syslog.write("Collision Distance: %f\n", value1); 
		}
        
		if (config.search("A", value1) &&
		    config.search("B", value2) ) {
			vfh.setVFHPlusParameter(value1, value2);
			vfh.getVFHPlusParameter(value1, value2);
			syslog.write("VFH Para.: %f, %f\n", value1, value2);
		}
        
		if (config.search("LowThreshold", value1) &&
		    config.search("HighThreshold", value2)) {
			vfh.setVFHThreshold(value1, value2);
			syslog.write("VFH Threshold: low:%f, high: %f\n", value1, value2);
		}
		
		if (config.search("u1", value1) &&
		    config.search("u2", value2) && 
		    config.search("u3", value3)) {
			vfh.setCostFuncParameter(value1, value2, value3);
			vfh.getCostFuncParameter(value1, value2, value3);
			syslog.write("u1: %f, u2: %f, u3: %f\n", value1, value2, value3);
		}
        
		/* Configuration is done, start measurement */
		vfh.start();
	}

	/* Geodesic class */
	const Geodesic &geod = Geodesic::WGS84;

	/* Path point class */
	PathPoints path;

	// Debug section
#ifdef DEBUG
	path.add(23.69481, 120.53631);
	path.add(23.69482, 120.53632);
	path.add(23.69483, 120.53633);
#endif

	/* Error counter */
	int cmdErrorCounter = 0;
	int URGErrorCounter = 0;

	/* Navigation toggle flag */
	bool navigationStart = false;

	/* Temp. Variable */
	bool updated = false;
	double latUpdated, lonUpdated;

	int speedValue = 0;
	int steerValue = 0;
	int steerValue_p = 0;
	double latTarget, lonTarget;
	double distance, azi1, azi2;
	double directionError;
	double commandAngle;

	/* Entering main loop */
	while(!quit.load(std::memory_order_relaxed)) {
		syslog.flush();

		/* Update VFH algorithm */
		if (!vfh.update()) {
			//quit.store(1, std::memory_order_relaxed);
			syslog.write("Error: Main: read LRF: %s\n", vfh.what());
			syslog.write("Error: Main: stop LRF measuremnet.\n");
			vfh.stop_measurement();
			syslog.write("Error: Main: start LRF measuremnet.\n");
			vfh.start_measurement(Urg_driver::Distance);
			syslog.write("Error: Main: LRF measurement started.\n");
			URGErrorCounter++;
		} else {
			URGErrorCounter = 0;
		}
		
		// Read data from gps thread
		pthread_mutex_lock(&(MData.mutex));
		unsigned char statusBit = MData.status;
		double yaw = MData.yaw;
		double latMeasured = MData.latitude;
		double lonMeasured = MData.longitude;
		CmtGpsStatus gpsStatus = MData.gpsStatus;
		pthread_mutex_unlock(&(MData.mutex));

		// Debug section
#ifdef DEBUG
		yaw = 0.0;
		latMeasured = 23.6948;
		lonMeasured = 120.5363;
#endif

		/* Check the status of gps. If status OK, turn led2 on. */
		if (statusBit == 7)
			led2.value(1);
		else
			led2.value(0);
	
		/* Emergency Button */
		if (btn1.value() == 1) {
			syslog.write("System: Emergency Stop\n");
			drive.setSteer(0);
			drive.setSpeed(0);
			navigationStart = false;

			continue;
		}

		/* Reading command */
		PacketType packetType = communicator.readCommand(&packet);

		/* Error command or no data */
		if (packetType == ERROR_CMD || packetType == TIMEOUT) {
			cmdErrorCounter++;
		} else {
			cmdErrorCounter = 0;
		}

		/* Resolving command */
		switch(packetType) {
		case RC_COMMAND:
			speedValue = (signed char)packet.field.speed;
			steerValue = (signed char)packet.field.steer;
			break;

		case READ_PATH:
			syslog.write("System: Read path from file:%s\n", packet.field.pathName);
			if (path.readFromFile(packet.field.pathName) > 0) {
				packet.field.packetType = ACK_OK;
			} else {
				packet.field.packetType = ACK_NOK;
			}
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case CLEAR_PATH:
			syslog.write("System: Clear Path!\n");
			path.clear();
			packet.field.packetType = ACK_OK;
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case SAVE_PATH:
			syslog.write("System: Save Path to:%s\n",
				packet.field.pathName);
			if (path.writeToFile(packet.field.pathName) > 0) {
				packet.field.packetType = ACK_OK;
			} else {
				packet.field.packetType = ACK_NOK;
			}
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case ASK_STATUS:
			syslog.write("System: Ask status\n");
			packet.field.packetType = ACK_STATUS;
			packet.field.latitude = latMeasured;
			packet.field.longitude = lonMeasured;
			updated = true;
			latUpdated = latMeasured;
			lonUpdated = lonMeasured;
			packet.field.yaw = yaw;
			packet.field.statusBit = statusBit;
			if (navigationStart) {
				packet.field.statusBit |= 0x08;
			} else {
				packet.field.statusBit &= ~0x08;
			}
			packet.field.totalPathPointNumber = path.size();
			packet.field.pathPointNumber = (unsigned char)(path.getCurrentIndex() + 1);
			communicator.sendCommand(&packet);
			break;

		case ASK_GPSSTATUS:
			syslog.write("System: Asking GPS Status\n");
			packet.field.packetType = ACK_GPSSTATUS;
			packet.field.gpsStatus = gpsStatus;
			communicator.sendCommand(&packet);
			break;

		case TOGGLE_NAV:
			syslog.write("System: Toggling Navigation!\n");
			if (!navigationStart) {
				if (path.empty()) {
					syslog.write("System: No path points exists, navigation cancelled\n");
					packet.field.packetType = ACK_NOK;
				} else {
					syslog.write("System: Start navigation.\n");
					packet.field.packetType = ACK_OK;

					packet.field.statusBit |= 0x08;
					navigationStart = true;
				}

			} else {
				syslog.write("System: Stop navigation.\n");
				packet.field.packetType = ACK_OK;
				navigationStart = false;

				navlog.write("\n\n");
				packet.field.statusBit &= ~0x08;
			}

			packet.field.pathPointNumber = (unsigned char)(path.getCurrentIndex() + 1);
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case RESET:
			syslog.write("System: Reset path point index!\n");

			if (path.setCurrentIndex(-1) == -2) {
				printf("unable to set index!\n");
			}
			navigationStart = false;

			packet.field.packetType = ACK_OK;
			packet.field.pathPointNumber = path.getCurrentIndex() + 1;
			packet.field.totalPathPointNumber = path.size();
			if (navlog.close() == -1) {
				syslog.write("System: Unable to close navigation log: Not open.\n");
			} else {
				syslog.write("System: Close navigation log.\n");
			}
				
			communicator.sendCommand(&packet);
			break;

		case SKIP:
			if (path.setCurrentIndex(path.getCurrentIndex() + 1) == -2) { // Error
				packet.field.packetType = ACK_NOK;
				syslog.write("System: Error: Unable to skip.\n");
			} else {
				packet.field.packetType = ACK_OK;
				syslog.write("System: Skip to the next path point.\n");
			}
			packet.field.pathPointNumber = (unsigned char)(path.getCurrentIndex() + 1);
			packet.field.totalPathPointNumber = path.size();

			communicator.sendCommand(&packet);
			break;

		case ADD_PATH:
			if (updated) {
				updated = false;
				syslog.write("System: Adding path point: %3.7f, %3.7f, total path number = %d\n", 
					latUpdated,
					lonUpdated,
					path.size());

				path.add(latUpdated, lonUpdated);
				packet.field.packetType = ACK_OK;
				packet.field.totalPathPointNumber = path.size();
			} else {
				syslog.write("System: Path point are not updated, aborted.\n");
				packet.field.packetType = ACK_NOK;
			}

			communicator.sendCommand(&packet);
			break;

		case REMOVE_PATH:
			syslog.write("System: Removing %d path point\n", packet.field.pathPointNumber);
			if (path.remove(packet.field.pathPointNumber) == 0) {
				packet.field.packetType = ACK_OK;
				packet.field.totalPathPointNumber = path.size();
			} else {
				packet.field.packetType = ACK_NOK;
				packet.field.totalPathPointNumber = path.size();
			}
			communicator.sendCommand(&packet);
			break;

		case SET_YAWOFFSET:
			syslog.write("System: Setting yaw angle offset: %f\n", packet.field.yaw);
			YawOffset = packet.field.yaw;
			packet.field.packetType = ACK_OK;
			communicator.sendCommand(&packet);
			break;

		default:
			break;
		}
		if (cmdErrorCounter >= 10) {
			speedValue = 0;
			steerValue = 0;
		}

		/* Start navigation */
		if (navigationStart) {
			/* Reset Command Error counter */
			cmdErrorCounter = 0;

			/* For the first time */
			if (path.getCurrentIndex() == -1) {
				path.setCurrentIndex(0);
				navlog.open();
				syslog.write("System: Open navigation log.\n");
			}
			
			/* Read target coor. from current index. */
			path.get(latTarget, lonTarget);

			/* Calculate geodesic distance and azimuth using GeographicLib */
			geod.Inverse(latMeasured, lonMeasured, latTarget, lonTarget, distance, azi1, azi2);
			if (distance < PositionError) {
				/* Read the next target coor. */
				if (path.getNext(latTarget, lonTarget) == -1) { // Reached the final
					syslog.write("System: Reaching the final path point, stop navigation and close the navigation log.\n");
					path.setCurrentIndex(-1);
					navigationStart = false;
					
					navlog.close();
					continue;
				}
			}
			
			/* Since azimuth is measured clockwise from meridian,
			 * it has to multiply by -1 in order to fit the coordinate system
			 * in Yun-Trooper II.
			 */	
		
			printf("Yaw + YawOffset = %f\n", yaw + YawOffset);
			directionError = (-azi1) - (yaw + YawOffset);
			/* IMPORTANT!!!! Unwrap the angle */
			if (directionError > 180)
				directionError -= 360;
			else if (directionError < -180)
				directionError += 360;

			/* VFH Plus algorithm */
			commandAngle = vfh.calculateDirection(directionError);

			if (commandAngle <= 180) { // c_t is available
				steerValue_p = steerValue;
				steerValue = (int)(commandAngle * SteeringGain);
				speedValue = (int)((MaxSpeed - MinSpeed) * (1.0 - vfh.getDensity()) + MinSpeed);
			} else { // c_t is not available
				speedValue = (int)MinSpeed;
				steerValue = steerValue_p;
			}

			// Collision detected
			if (vfh.collisionDetected(((double)steerValue) / SteeringGain))
				speedValue = 0;

			/* VFH Error! */
			if (URGErrorCounter >= 10) {
				speedValue = 0;
				steerValue = 0;
			}

			if (commandAngle > 180)
				led1.toggle();
			else
				led1.value(1);
#ifdef DEBUG
			printf("%02d,%02d,%+3.7f,%+3.7f,%+3.7f,%+3.7f,%+3.4f,%+3.4f,%3d,%3d,%3f\n",
				     path.getCurrentIndex()+1,	 // 1 -> current path number
				     path.size(),                // 2 -> total path number
				     latTarget,                  // 3 -> lat. of target
				     lonTarget,                  // 4 -> lon. of target
				     latMeasured,                // 5 -> lat. of current pos.
				     lonMeasured,                // 6 -> lon. of current pos.
				     yaw,                        // 7 -> current direction(yaw angle)
				     commandAngle,               // 8 -> direction calculated by VFH algorithm
				     speedValue,                 // 9 -> speed value
				     steerValue,                //10 -> steer value
				     directionError);
#else	
			/* Record all data during navigation */
			navlog.write("%02d,%02d,%+3.7f,%+3.7f,%+3.7f,%+3.7f,%+3.4f,%+3.4f,%3.4f,%3d,%3d\n",
				     path.getCurrentIndex()+1,	 // 1 -> current path number
				     path.size(),                // 2 -> total path number
				     latTarget,                  // 3 -> lat. of target
				     lonTarget,                  // 4 -> lon. of target
				     latMeasured,                // 5 -> lat. of current pos.
				     lonMeasured,                // 6 -> lon. of current pos.
				     yaw,                        // 7 -> current direction(yaw angle)
				     commandAngle,               // 8 -> direction calculated by VFH algorithm
				     directionError,		 // 9 -> direction error
				     speedValue,                 //10 -> speed value
				     steerValue);                //11 -> steer value
			navlog.flush();
#endif
		} else {
			led1.value(0);
		}

		/* Actuating motor with speed steer value */
		drive.setSpeed(speedValue);
		drive.setSteer(steerValue);
	}

	pthread_join(thread, NULL);
	syslog.write("Program Exit\n");
	syslog.close();
	navlog.close();
	exit(EXIT_FAILURE);
}	
