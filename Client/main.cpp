#include "communicator/communicator.h"
#include "communicator/packetclient.h"
#include "PathPoints/pathpoints.h"
#include "drive/drive.h"
#include "drive/gpio.h"
#include "MTiG/cmt3.h"
#include "VFH/vfh.h"
#include "logfile/logfile.h"

#include <Urg_driver.h>
#include <errno.h>
#include <atomic>

#define ALPHA 0.3
#define BETA 0.7
#define SKIP_INDEX 4
#define BODY_WIDTH 200 
#define STEERING_GAIN 1.5
#define MAX_SPEED 80 
#define POS_ERR_SQUARE 1

/* Atomic quit bit */
std::atomic_char quit(0);

/* Global data logger */
LogFile syslog;

using namespace YT;
using namespace xsens;
using namespace qrk;

struct MTiGData {
	pthread_mutex_t mutex;
	unsigned char status;
	double latitude;
	double longitude;
	double yaw;
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
	}
	
	int timeout = 1000;
	if (serial.setTimeoutMeasurement(timeout) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set timeout: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}

	/* Show available scenarios
	CmtScenario *scen = new CmtScenario[CMT_MAX_SCENARIOS_IN_MT + 1];
	serial.getAvailableScenarios(scen);
	for (int i=0; i<CMT_MAX_SCENARIOS_IN_MT+1; i++) {
		printf("m_type:%d, m_lable:%s\n", scen[i].m_type, scen[i].m_label);
	}
	*/

	/* Set MTiG scenario */
	if (serial.setScenario(1) != XRV_OK) {
		syslog.write("Error: MTiG: %d occured during set scenario: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}
	
	CmtDeviceMode mode(CMT_OUTPUTMODE_STATUS |
			   CMT_OUTPUTMODE_ORIENT |
			   CMT_OUTPUTMODE_POSITION |
			   CMT_OUTPUTMODE_GPSPVT_PRESSURE,

			   CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT |
			   CMT_OUTPUTSETTINGS_ORIENTMODE_EULER |
			   CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632, 
			   20);
	if (serial.setDeviceMode(mode, false, CMT_DID_BROADCAST)) {
		syslog.write("Error: MTiG: %d occured during set device mode: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
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

	if (serial.gotoMeasurement()) {
		syslog.write("Error: MTiG: %d occured during go to measurment: %s\n",
			serial.getLastResult(),
			xsensResultText(serial.getLastResult()));
		quit.store(1, std::memory_order_relaxed);
	}

	while (!quit.load(std::memory_order_relaxed)) {
		if (serial.waitForDataMessage(&reply) != XRV_OK) {
			syslog.write("Error: MTiG: %d occured during read data message: %s\n",
				serial.getLastResult(),
				xsensResultText(serial.getLastResult()));
			quit.store(1, std::memory_order_relaxed);
		}
		pthread_mutex_lock(&(data->mutex));
		data->status = reply.getStatus();
		data->yaw = reply.getOriEuler().m_yaw;
		data->latitude = reply.getPositionLLA().m_data[0];
		data->longitude = reply.getPositionLLA().m_data[1];
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
	navlog.open();

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
	steerPara.deadzone = 20;
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
	struct MTiGData gpsData;
	pthread_mutex_init(&(gpsData.mutex), NULL);
	pthread_t thread;
	if (pthread_create(&thread, NULL, GPSRead, (void*)&gpsData) < 0) {
		syslog.write("Error: Main: Thread Creation failed : %s\n", strerror(errno));
		quit.store(1, std::memory_order_relaxed);
		quit = 1;
	}

	/* Setting up LRF */
	Urg_driver urg;
	if (!urg.open("/dev/ttyACM0")) {
		syslog.write("Error: Main: Unable to open LRF: %s\n", urg.what());
		quit.store(1, std::memory_order_relaxed);
		quit = 1;
	}
	int min_step, max_step;
	urg.start_measurement(Urg_driver::Distance);
	min_step = urg.min_step();
	max_step = urg.max_step();

	std::vector<double> angle;
	for (int i=min_step; i<=max_step; i+=SKIP_INDEX) {
		angle.push_back(urg.step2deg(i));
	}

	/* VFH algorithm */
	VFH vfh;
	vfh.setAngleIndex(angle);
	vfh.setDetectionZone(60.0, 1500);

	/* Path point class */
	PathPoints path;

	/* Debug!!!
	path.add(23.69481, 120.5363001);
	*/

	/* Error counter */
	int errorCounter = 0;
	int collisionCounter = 0;

	/* Navigation toggle flag */
	bool Flag_Nav = false;
	
	/* Temp. Variable */
	double latTarget, lonTarget;
	int speedValue = 0;
	int steerValue = 0;

	while(!quit.load(std::memory_order_relaxed)) {
		syslog.flush();

		/* Reading LRF data */
		std::vector<long> data;
		std::vector<long> reducedData;
		long time_stamp = 0;
		if (!urg.get_distance(data, &time_stamp)) {
			//quit.store(1, std::memory_order_relaxed);
			std::string errStr = urg.what();
			syslog.write("Error: Main: read LRF: %s\n", errStr.c_str());
			syslog.write("Error: Main: stop LRF measuremnet.\n");
			urg.stop_measurement();
			syslog.write("Error: Main: start LRF measuremnet.\n");
			urg.start_measurement(Urg_driver::Distance);
		}
		
		// Reducing the data and update the collision counter
		collisionCounter = 0;
		for (int j=urg.step2index(min_step); j<urg.step2index(max_step); j+=SKIP_INDEX) {
			if (data[j] > 4000 || data[j] < 20)
				data[j] = 4000;
			reducedData.push_back(data[j] - BODY_WIDTH);

			if ((data[j] - BODY_WIDTH) < 0 )
				collisionCounter++;
		}

		// Update VFH
		vfh.update(reducedData);

		// Read data from gps thread
		pthread_mutex_lock(&(gpsData.mutex));
		unsigned char gpsStatus = gpsData.status;
		double yaw = gpsData.yaw;
		double latMeasured = gpsData.latitude;
		double lonMeasured = gpsData.longitude;
		pthread_mutex_unlock(&(gpsData.mutex));

		/* Debug!!!!
		yaw = 0.0;
		latMeasured = 23.6948000;
		lonMeasured = 120.5363000;
		*/

		/* 
		   Check the status of gps. If status OK, turn led2 on.
		*/
		if (gpsStatus == 7)
			led2.value(1);
		else
			led2.value(0);
	
		/* Emergency Button */
		if (btn1.value() == 1) {
			syslog.write("System: Emergency Stop\n");
			drive.setSteer(0);
			drive.setSpeed(0);
			Flag_Nav = false;

			// Reset steering deadzone
			steerPara.deadzone = 20;
			drive.setSteerParameter(steerPara);

			continue;
		}

		/* Reading command */
		PacketType packetType = communicator.readCommand(&packet);

		/* Error command or no data */
		if (packetType == ERROR_CMD || packetType == TIMEOUT) {
			errorCounter++;
		} else {
			errorCounter = 0;
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
			packet.field.yaw = yaw;
			packet.field.statusBit = gpsStatus;
			if (Flag_Nav) {
				packet.field.statusBit |= 0x08;
			} else {
				packet.field.statusBit &= ~0x08;
			}
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case TOGGLE_NAV:
			syslog.write("System: Toggling Navigation!\n");
			if (!Flag_Nav) {
				syslog.write("System: Start nav.\n");
				Flag_Nav = true;

				// Set smaller steering deadzone
				steerPara.deadzone = 10;
				drive.setSteerParameter(steerPara);

				packet.field.statusBit |= 0x08;
				/* For the first time */
				if (path.getCurrentIndex() == -1) {
					path.getNext(latTarget, lonTarget);
				}

			} else {
				syslog.write("System: Stop nav.\n");
				Flag_Nav = false;

				// Reset steering deadzone
				steerPara.deadzone = 20;
				drive.setSteerParameter(steerPara);

				navlog.write("\n\n");
				packet.field.statusBit &= ~0x08;
			}

			packet.field.packetType = ACK_OK;
			packet.field.pathPointNumber = (unsigned char)path.getCurrentIndex() + 1;
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case RESET:
			syslog.write("System: Reset!\n");
			packet.field.packetType = ACK_OK;
			packet.field.totalPathPointNumber = path.size();
			communicator.sendCommand(&packet);
			break;

		case ADD_PATH:
			syslog.write("System: Adding path point: %3.7f, %3.7f, total path number = %d\n", 
				packet.field.latitude,
				packet.field.longitude,
				path.size());

			path.add(packet.field.latitude, packet.field.longitude);

			packet.field.packetType = ACK_OK;
			packet.field.totalPathPointNumber = path.size();
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

		default:
			break;
		}

		if (errorCounter >= 10) {
			speedValue = 0;
			steerValue = 0;
		}

		/* Start navigation */
		if (Flag_Nav) {
			/* Reset Error counter */
			errorCounter = 0;

			if (path.empty()) {
				syslog.write("System: Path does not exist, navigation cancelled\n");
				Flag_Nav = false;
				continue;
			}
				
			double m_x, m_y, distSquare;
			m_x = PathPoints::Lat2Meter(latMeasured, latTarget);
			m_y = PathPoints::Lon2Meter(latMeasured, lonMeasured, lonTarget);
			distSquare = m_x*m_x + m_y*m_y;

			if (distSquare < POS_ERR_SQUARE) {
				if (!path.getNext(latTarget, lonTarget)) {
					syslog.write("System: Reaching final path point, stop navigation.\n");
					path.setCurrentIndex(0);
					Flag_Nav = false;

					// Reset steering deadzone
					steerPara.deadzone = 20;
					drive.setSteerParameter(steerPara);
				}
			}

			double targetDirection = -(atan2(m_y, m_x)) * (180/M_PI);
			double directionError = targetDirection - yaw;
			
			/* IMPORTANT!!!! Unwrap the angle */
			if (directionError > 180)
				directionError -= 360;
			else if (directionError < -180)
				directionError += 360;
			
			int obstacleDetectedValue;
			if (vfh.obstacleDetected()) {
				obstacleDetectedValue = 1;
				double max = 0.0;
				double commandAngle = 0.0;
				for (size_t i=0; i<vfh.sector.size(); i++) {
					double diff = (directionError - vfh.sector[i].angle);
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
				steerValue = (int)(commandAngle * STEERING_GAIN);
                        
				if (vfh.sector.size() == 0) {
					speedValue = 0;
				} else {
					speedValue = (int)((1.0 - vfh.getDensity()) * MAX_SPEED);
				}
			} else {
				obstacleDetectedValue = 0;
				steerValue = (int)(directionError * STEERING_GAIN);
				speedValue = MAX_SPEED;
			}

			// Collision Detected
			if (collisionCounter >= 5) {
				obstacleDetectedValue = -1;
				speedValue = 0;
				steerValue = 0;
				led1.toggle();
			} else {
				led1.value(1);
			}

			/* Nav. log structure:
			   1 -> current path number
			   2 -> total path number
			   3 -> lat. of target
			   4 -> lon. of target
			   5 -> lat. of current pos.
			   6 -> lon. of current pos.
			   7 -> target direction
			   8 -> current direction(yaw angle)
			   9 -> speed value
			   10-> steer value
			   11-> obstacle (1 for detected, 0 for nothing);
			*/
			navlog.write("%02d,%02d,%+3.7f,%+3.7f,%+3.7f,%+3.7f,%+3.4f,%+3.4f,%3d,%3d,%d\n",
				     path.getCurrentIndex()+1,
				     path.size(),
				     latTarget,
				     lonTarget,
				     latMeasured,
				     lonMeasured,
				     directionError,
				     yaw,
				     speedValue,
				     steerValue,
				     obstacleDetectedValue);
			navlog.flush();
		} else {
			led1.value(0);
		}

		/* Actuating motor with speed steer value */
		drive.setSpeed(speedValue);
		drive.setSteer(steerValue);
		//printf("speedValue = %3d, steerValue = %3d\n", speedValue, steerValue);
	}
	pthread_join(thread, NULL);
	syslog.writeErr("Program Exit", errno);
	syslog.close();
	navlog.close();
	return -1;
}	
