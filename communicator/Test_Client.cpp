#include "communicator.h"
#include "packetclient.h"
#include "drive/drive.h"
#include <stdlib.h>

using namespace YT;

int main(void)
{
	PacketClient packet;
	Drive drive;
	Communicator comm;
	comm.open("/dev/ttyO4", 57600);
	comm.flushData();

	while(1) {
		PacketType packetType = comm.readCommand(&packet);
		if (packetType == CLEAR_PATH) {
			printf("CLEAR!\n");
			packet.field.packetType = ACK_OK;
			packet.field.totalPathPointNumber = 0;
			comm.flushData();
			comm.sendCommand(&packet);
		} else if (packetType == ASK_STATUS) {
			printf("ASK STATUS\n");
			srand(time(NULL));
			packet.field.packetType = ACK_STATUS;
			packet.field.latitude = (double) rand();
			packet.field.longitude = (double) rand();
			packet.field.yaw = (double) rand();
			packet.field.statusBit = 0x07;
			/*
			packet.packingData();
			for (int i=0; i<PACKET_SIZE; i++) {
				printf("%X ", packet.rawData[i]);
			}printf("\n");
			*/
			comm.flushData();
			comm.sendCommand(&packet);
		} else if (packetType == RC_COMMAND) {
			signed char speed = packet.field.speed;
			signed char steer = packet.field.steer;
			drive.setSpeed(speed);
			drive.setSteer(steer);
			printf("speed : %d, steer : %d\n", speed, steer);
		} else if (packetType == ADD_PATH) {
			printf("Received lat:%3.6f, lon:%3.6f\n", 
			packet.field.latitude,
			packet.field.longitude);
			packet.field.packetType = ACK_OK;
			packet.field.totalPathPointNumber = 1;
			comm.flushData();
			comm.sendCommand(&packet);

		} else if (packetType == TIMEOUT) {
			//printf("Timeout...\n");
		} else if (packetType == ERROR_CMD) {
			printf("Error...\n");
		} else {
			printf("WTF!?\n");
		}
		usleep(40000);
	}
	return 0;
}
	

