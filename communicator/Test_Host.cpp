#include <signal.h>
#include <sys/time.h>
#include <string.h>

#include "joystick/controller.h"
#include "packethost.h"
#include "communicator.h"

int quit = 0;

void alarm_handler(int signo){};
void ctrlc_handler(int signo)
{
	printf("Interrupt\n");
	quit = 1;
}

using namespace YT;

int main(void)
{
	/* Setting up signal handler */
	struct sigaction act;
	act.sa_handler = alarm_handler;
	sigaction(SIGALRM, &act, NULL);

	act.sa_handler = ctrlc_handler;
	sigaction(SIGINT, &act, NULL);

	/* Setting interval timer */
	struct itimerval delay;
	delay.it_value.tv_sec = 0;
	delay.it_value.tv_usec = 50000;
	delay.it_interval.tv_sec = 0;
	delay.it_interval.tv_usec = 50000;

	/* XBox controller */
	Controller xbox("/dev/input/js0", NONBLOCK);
	Buttons btns;

	/* Setting communicator */
	Communicator communicator;
	communicator.open("/dev/ttyUSB0", 57600);
	communicator.flushData();

	PacketHost command;

	if (setitimer(ITIMER_REAL, &delay, NULL)) {
		perror("setitimer");
		return -1;
	}

	while(!quit) {
		pause();
		xbox.readButtons(&btns);
		if (btns.BtnA()) {
			command.field.packetType = CLEAR_PATH;
			
			pause();
			communicator.sendCommand(&command);
			pause();
			if (communicator.readCommand(&command) == ACK_CLEAR_PATH) {
				printf("Ack. message received!\n");
			} else {
				printf("No ack. message...\n");
			}
			continue;

		} else if (btns.BtnB()) {
			command.field.packetType = ASK_STATUS;
			pause();
			communicator.sendCommand(&command);
			pause();
			if (communicator.readCommand(&command) == ACK_STATUS) {
				printf("Ack. message received!\n");
				printf("Lat = %f, Lon = %f, Yaw = %f\n", 
					command.field.latitude,
					command.field.longitude,
					command.field.yaw);
			} else {
				printf("No ack. message...\n");
			}

		} else {
			signed char steer = -(btns.AxisX() * 100) / 32767;
			signed char speed = -(btns.AxisZ2() * 100) / 32767;
			command.field.packetType = RC_COMMAND;
			command.field.speed = speed;
			command.field.steer = steer;
			communicator.sendCommand(&command);
		}
	}
	return 0;
}



