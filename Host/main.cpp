#include <sys/time.h>
#include <signal.h>
#include <iostream>
#include <iomanip>

#include "joystick/controller.h"
#include "communicator/communicator.h"
#include "communicator/packethost.h"

using namespace YT;
using namespace std;

/* Period of interval timer (in us) */
#define PERIOD 100000

/* Macros for start and stop interval timer */
#define StartTimer(delay);  delay.it_value.tv_sec = 0;\
			    delay.it_value.tv_usec = PERIOD;\
			    delay.it_interval.tv_sec = 0;\
			    delay.it_interval.tv_usec = PERIOD;\
			    setitimer(ITIMER_REAL, &delay, NULL);

#define StopTimer(delay);   delay.it_value.tv_sec = 0;\
			    delay.it_value.tv_usec = 0;\
			    delay.it_interval.tv_sec = 0;\
			    delay.it_interval.tv_usec = 0;\
			    setitimer(ITIMER_REAL, &delay, NULL);
int quit = 0;

void alarm_handler(int signo){}

void ctrlc_handler(int signo)
{
	cout << "Interrupt\n";
	quit = 1;
}


int main(void)
{
	/* Setting up signal handler */
	struct sigaction act;
	act.sa_handler = alarm_handler;
	sigaction(SIGALRM, &act, NULL);

	act.sa_handler = ctrlc_handler;
	sigaction(SIGINT, &act, NULL);

	/* Setting up interval timer */
	struct itimerval delay;
	StartTimer(delay);

	/* Setting up xbox controller */
	Controller xbox("/dev/input/js0", NONBLOCK);
	Buttons btns;

	/* Setting up communicator */
	Communicator communicator;
	communicator.open("/dev/ttyUSB0", 38400);
	communicator.flushData();
	PacketType ack;
	PacketHost packet;

	/* Start the interval timer */
	if (setitimer(ITIMER_REAL, &delay, NULL)) {
		perror("set interval timer");
		exit(EXIT_FAILURE);
	}
	
	/* Main loop */
	while(!quit) {
		pause();
		xbox.readButtons(&btns);

		/* Query of status can be triggered at any time */
		if (btns.BtnLB()) {
			cout << "Ask:\tGetting status\n\n";
			packet.field.packetType = ASK_STATUS;
			communicator.sendCommand(&packet);
			pause();

			cout << "Received:\n";
			if (communicator.readCommand(&packet) == ACK_STATUS) {
				     cout << "\tLat: " << fixed << setprecision(6) << packet.field.latitude
				     << " Lon: " << packet.field.longitude
				     << " Yaw: " << packet.field.yaw << "\n";
				
				cout << "\t";
				if (packet.field.statusBit & 0x01)
					cout << "Self Test:  Passed.\n";
				else
					cout << "Self Test:  Failed.\n";

				cout << "\t";
				if (packet.field.statusBit & 0x02)
					cout << "XKF:        Valid.\n";
				else 
					cout << "XKF:        Invalid.\n";

				cout << "\t";
				if (packet.field.statusBit & 0x04)
					cout << "GPS:        Fixed\n";
				else 
					cout << "GPS:        Not Fixed.\n";

				cout << "\t";
				if (packet.field.statusBit & 0x08)
					cout << "Navigation: in Progress.\n";
				else
					cout << "Navigation: Stopped.\n";

				cout << "\t";
				cout << "Current path point number: "
				     << (signed int)packet.field.pathPointNumber << "\n";
			
				cout << "\t";
				cout << "Total path point number:   "
				     << (signed int)packet.field.totalPathPointNumber << "\n";
				cout << "\n";
			} else {
				cout << "\tError: No ack message received...\n";
			}

		/* Command mode, which requires LT pressed. */
		} else if (btns.AxisLT() > 20000) {

			StopTimer(delay);

			/* Adding the coordinate received from querying as the path points. */
			if (btns.BtnRB()) {
				cout << "Ask:\tAdding path point\n"
				     << "Lat: " << packet.field.latitude << " Lon: " << packet.field.longitude << "\n\n";

				packet.field.packetType = ADD_PATH;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);

				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: Total path point number: " << packet.field.totalPathPointNumber << "\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";

			/* Toggle Navigation. */
			} else if (btns.BtnXBOX()) {
				cout << "Ask:\tToggleNavigation\n\n";
				packet.field.packetType = TOGGLE_NAV;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);

				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tCurrent path point number: "
					     << packet.field.pathPointNumber << "\n";
					cout << "\tTotal path point number:   "
					     << packet.field.totalPathPointNumber << "\n";
					if (packet.field.statusBit & 0x08)
						cout << "\tStart navigation.\n";
					else 
						cout << "\tStop navigation.\n";
				} else if (ack == ACK_NOK) {
					cout << "\tError: No path point exist, navigation cancelled.\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";

			/* Skip to the next current path target. */
			} else if (btns.BtnStart()) {
				cout << "Ask:\tSkip to the next path point\n\n";
				packet.field.packetType = SKIP;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);
				
				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: Skip to the next path point.\n"
					     << "\t         Current path point number: "
					     << packet.field.pathPointNumber << "\n"
					     << "\t         Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
        
				} else if (ack == ACK_NOK) {
					cout << "\tError: Unable to skip.\n"
					     << "\t       Current path point number: "
					     << packet.field.pathPointNumber << "\n"
					     << "\t       Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";

			/* Reset */
			} else if (btns.BtnBack()) {
				cout << "Ask:\tReset path point index\n\n";
				packet.field.packetType = RESET;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);
				
				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: Reset path point index.\n"
					     << "\t         Current path point number: "
					     << packet.field.pathPointNumber << "\n"
					     << "\t         Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
        
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";
			/* Clear all path points. */
			} else if (btns.BtnY()) {
				cout << "Ask:\tClear all path points\n\n";
				packet.field.packetType = CLEAR_PATH;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);

				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess:";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";
        
			/* Read path from file. */
			} else if (btns.BtnA()) {
				cout << "Ask:\tRead path from file: Please enter the name:\n";
				packet.field.packetType = READ_PATH;
				scanf("%s", packet.field.pathName);
				cout << "\n";

				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);

				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: File read.\n"
					     << "\t         Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
        
				} else if (ack == ACK_NOK) {
					cout << "\tError: File not read.\n"
					     << "\t       Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";
			
			/* Save path points into file. */
			} else if (btns.BtnX()) {
				cout << "Ask:\tSave path to file: Please enter the name:\n";
				packet.field.packetType = SAVE_PATH;
				scanf("%s", packet.field.pathName);
				cout << "\n";

				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);

				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: File saved.\n"
					     << "\t         Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
        
				} else if (ack == ACK_NOK) {
					cout << "\tError: File not saved.\n"
					     << "\t       Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";

			/* Remove path point. */
			} else if (btns.BtnB()) {
				cout << "Ask:\tRemove path point: Please enter the index of any path point:\n";
				packet.field.packetType = REMOVE_PATH;
				int inputValue;
				scanf("%d", &inputValue);

				packet.field.pathPointNumber = (unsigned char)inputValue;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				ack = communicator.readCommand(&packet);
				cout << "Received:\n";
				if (ack == ACK_OK) {
					cout << "\tSuccess: Path point removed.\n"
					     << "\t         Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
        
				} else if (ack == ACK_NOK) {
					cout << "\tError: Path point not removed.\n"
					     << "\t       Total path point number: "
					     << packet.field.totalPathPointNumber << "\n";
				} else {
					cout << "\tError: No ack message received...\n";
				}
				cout << "\n";
			}

			StartTimer(delay);

		/* LT is released */
		} else {
			signed char steer = -(btns.AxisX() * 100) / 32767;
			signed char speed = -(btns.AxisZ2() * 100) / 32767;
			packet.field.packetType = RC_COMMAND;
			packet.field.speed = speed;
			packet.field.steer = steer;
			communicator.sendCommand(&packet);
		}
	}
	return 0;
}
