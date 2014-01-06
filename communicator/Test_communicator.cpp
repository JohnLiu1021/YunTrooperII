#include "communicator.h"
#include <stdlib.h>

using namespace YT;

int main(void)
{
	Packet packet;
	Communicator comm;
	comm.open("/dev/ttyO4", 57600);
	comm.flushData();

	while(1) {
		PacketType packetType = comm.readCommand(&packet);
		if (packetType == CLEAR_PATH) {
			printf("CLEAR!\n");
			packet.packetType = ACK_CLEAR_PATH;
			comm.sendCommand(&packet);
		} else if (packetType == ASK_STATUS) {
			printf("ASK STATUS\n");
			srand(time(NULL));
			packet.latitude = (double)rand();
			packet.longitude =(double)rand();
			packet.yaw = (double)rand();
			packet.statusBit = (double)rand();
			packet.packetType = ACK_STATUS;
			packet.packingData();
			for (int i=0; i<PACKET_SIZE; i++) {
				printf("%X ", packet.rawData[i]);
			}printf("\n");
			comm.sendCommand(&packet);
		} else if (packetType == RC_COMMAND) {
			signed char speed = packet.speed;
			signed char steer = packet.steer;
			printf("speed : %d, steer : %d\n", speed, steer);
		} else if (packetType == ERROR_CMD) {
			printf("Error...\n");
		}
		usleep(50000);
	}
	return 0;
}
	

