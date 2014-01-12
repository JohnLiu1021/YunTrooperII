#include "communicator.h"
#include "packetclient.h"
#include <stdlib.h>

using namespace YT;

int main(void)
{
	PacketClient packet;
	Communicator comm;
	comm.open("/dev/ttyO4", 57600);
	comm.flushData();

	while(1) {
		PacketType packetType = comm.readCommand(&packet);
		if (packetType == CLEAR_PATH) {
			printf("CLEAR!\n");
			packet.field.packetType = ACK_CLEAR_PATH;
			comm.sendCommand(&packet);
		} else if (packetType == ASK_STATUS) {
			printf("ASK STATUS\n");
			srand(time(NULL));
			packet.field.latitude = (double)rand();
			packet.field.longitude =(double)rand();
			packet.field.yaw = (double)rand();
			packet.field.statusBit = (double)rand();
			packet.field.packetType = ACK_STATUS;
			packet.packingData();
			for (int i=0; i<PACKET_SIZE; i++) {
				printf("%X ", packet.rawData[i]);
			}printf("\n");
			comm.sendCommand(&packet);
		} else if (packetType == RC_COMMAND) {
			signed char speed = packet.field.speed;
			signed char steer = packet.field.steer;
			printf("speed : %d, steer : %d\n", speed, steer);
		} else if (packetType == ERROR_CMD) {
			printf("Error...\n");
		}
	}
	return 0;
}
	

