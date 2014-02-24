#include <sys/time.h>
#include <signal.h>

#include "joystick/controller.h"
#include "communicator/communicator.h"
#include "communicator/packethost.h"

using namespace YT;

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
	printf("Interrupt\n");
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
			printf("Getting status.\n");
			packet.field.packetType = ASK_STATUS;
			communicator.sendCommand(&packet);
			pause();
			if (communicator.readCommand(&packet) == ACK_STATUS) {
				printf("Lat = %3.6f, Lon = %3.6f, Yaw = %3.6f\n", 
					packet.field.latitude,
					packet.field.longitude,
					packet.field.yaw);
				printf("Status:\n");
				if (packet.field.statusBit & 0x01)
					printf("Self Test Passed.\n");
				else
					printf("Self Test Failed.\n");

				if (packet.field.statusBit & 0x02)
					printf("XKF Valid.\n");
				else 
					printf("XKF Invalid.\n");

				if (packet.field.statusBit & 0x04)
					printf("GPS Fixed\n");
				else 
					printf("GPS Not Fixed.\n");

				if (packet.field.statusBit & 0x08)
					printf("Navigation in Progress.\n");
				else
					printf("Navigation Stopped.\n");
			
				printf("Total path point number = %d\n",
					packet.field.totalPathPointNumber);
			}

		/* Command mode, which requires pressing of LT */
		} else if (btns.AxisLT() > 20000) {

			StopTimer(delay);

			if (btns.BtnRB()) {
				printf("Adding path point : Lat = %3.6f, Lon = %3.6f\n",
					packet.field.latitude,
					packet.field.longitude);
				packet.field.packetType = ADD_PATH;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				if (communicator.readCommand(&packet) == ACK_OK) {
					printf("Total path point number : %d\n",
						packet.field.totalPathPointNumber);
				} else {
					printf("No ack message received...\n");
				}

			} else if (btns.BtnStart()) {
				printf("Toggle navigation.\n");
				packet.field.packetType = TOGGLE_NAV;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				if (communicator.readCommand(&packet) == ACK_OK) {
					printf("Total path point number : %d\n",
						packet.field.totalPathPointNumber);
					printf("Current path point number : %d\n",
						packet.field.pathPointNumber);
					if (packet.field.statusBit & 0x08)
						printf("Start navigation.\n");
					else 
						printf("Stop navigation.\n");
				} else {
					printf("No ack message received...\n");
				}

			} else if (btns.BtnY()) {
				printf("Clear all path points\n");
				packet.field.packetType = CLEAR_PATH;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				if (communicator.readCommand(&packet) == ACK_OK) {
					printf("Current total path point number : %d\n",
						packet.field.totalPathPointNumber);
				} else {
					printf("No ack message received...\n");
				}
        
			} else if (btns.BtnA()) {
				packet.field.packetType = READ_PATH;
				printf("Please enter the file name of path you want to read: \n");
				scanf("%s", packet.field.pathName);

				communicator.sendCommand(&packet);
				usleep(PERIOD);
				PacketType ack = communicator.readCommand(&packet);
				if (ack == ACK_OK) {
					printf("File successfully read.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else if (ack == ACK_NOK) {
					printf("File reading failed.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else {
					printf("No ack message received...\n");
				}
			
			} else if (btns.BtnX()) {
				packet.field.packetType = SAVE_PATH;
				printf("Please enter the file name of the path you want to save: \n");
				scanf("%s", packet.field.pathName);

				communicator.sendCommand(&packet);
				usleep(PERIOD);
				PacketType ack = communicator.readCommand(&packet);
				if (ack == ACK_OK) {
					printf("File successfully saved.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else if (ack == ACK_NOK) {
					printf("File saving failed.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else {
					printf("No ack message received...\n");
				}
			} else if (btns.BtnB()) {
				packet.field.packetType = REMOVE_PATH;
				printf("Please enter the number of point you want to remove: \n");
				int inputValue;
				scanf("%d", &inputValue);

				packet.field.pathPointNumber = (unsigned char)inputValue;
				communicator.sendCommand(&packet);
				usleep(PERIOD);
				PacketType ack = communicator.readCommand(&packet);
				if (ack == ACK_OK) {
					printf("Point successfully removed.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else if (ack == ACK_NOK) {
					printf("Point removing failed.\n");
					printf("Total path point number: %d\n",
						packet.field.totalPathPointNumber);
        
				} else {
					printf("No ack message received...\n");
				}
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
