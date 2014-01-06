#ifndef YT_PACKET_H
#define YT_PACKET_H 1

#include <string.h> // memcpy

namespace YT {

enum PacketType{
	/* ACK_NOK : Invalid requirement
	   Length = 0
	*/
	ACK_NOK 	= 0x99,

	/* ACK_READ_PATH : Return the number of path point read from the file 
	   Length = 1
	   packet[4] = Number of path point been read from the file
	*/
	ACK_READ_PATH	= 0x91,
	
	/* ACK_SAVE_PATH : Return the number of path point being saving in the file
	   Length = 1
	   packet[4] = number of path point been saving in the file
	*/
	ACK_SAVE_PATH	= 0x92,

	/* ACK_STATUS : Return the status of the sensor
	   Length = 24 
	   packet[4..11] = latitude
	   packet[12..19] = longitude
	   packet[20..27] = yaw
	   packet[28] = Sensor status bit
	*/
	ACK_STATUS	= 0x93,

	/* ACK_START_NAV : Return the path point number
	   Length = 1
	   packet[4] = Number of path point
	*/
	ACK_START_NAV	= 0x94,

	/* ACK_PAUSE_NAV : Return the remaining path point number
	   Length = 1
	   packet[4] = Number of remaining path point
	*/
	ACK_PAUSE_NAV	= 0x95,

	/* ACK_CLEAR_PATH: Return clear ok
	   Length = 0
	*/
	ACK_CLEAR_PATH	= 0x96,

	/* ACK_RESET : Reset the program
	   Length = 0;
	*/
	ACK_RESET	= 0x97,

	TIMEOUT		= 0,
	ERROR_CMD	= 0xFF,
	RC_COMMAND	= 0x02,
	READ_PATH	= 0x06,
	CLEAR_PATH	= 0x16,
	SAVE_PATH	= 0x26,
	ASK_STATUS	= 0x1A,
	START_NAV	= 0xBB,
	PAUSE_NAV	= 0xB1,
	RESET		= 0xDD,
};

enum Constant {
	PACKET_SIZE = 30
};

class Packet {
public:
	char speed;
	char steer;
	char pathName[255];
	double latitude;
	double longitude;
	double yaw;
	int pathPointNumber;
	int statusBit;
	unsigned char rawData[PACKET_SIZE];
	PacketType packetType;
	
	int packingData()
	{
		_addHeader(rawData);

		switch(packetType) {
		case ACK_NOK:
			rawData[2] = 0;
			rawData[3] = ACK_NOK;
			break;
        
		case ACK_READ_PATH:
			rawData[2] = 1;
			rawData[3] = ACK_READ_PATH;
			rawData[4] = pathPointNumber;
			break;
        
		case ACK_SAVE_PATH:
			rawData[2] = 1;
			rawData[3] = ACK_SAVE_PATH;
			rawData[4] = pathPointNumber;
			break;
        
		case ACK_STATUS:
			rawData[2] = 25;
			rawData[3] = ACK_STATUS;
			memcpy(rawData+4, &(latitude), 8);
			memcpy(rawData+12, &(longitude), 8);
			memcpy(rawData+20, &(yaw), 8);
			rawData[28] = statusBit;
			break;
        
		case ACK_START_NAV:
			rawData[2] = 1;
			rawData[3] = ACK_START_NAV;
			rawData[4] = pathPointNumber;
			break;
        
		case ACK_PAUSE_NAV:
			rawData[2] = 1;
			rawData[3] = ACK_PAUSE_NAV;
			rawData[4] = pathPointNumber;
			break;
        
		case ACK_CLEAR_PATH:
			rawData[2] = 0;
			rawData[3] = ACK_CLEAR_PATH;
			printf("This is it bro!\n");
			break;
        
		case ACK_RESET:
			rawData[2] = 0;
			rawData[3] = ACK_RESET;
			break;
        
		default:
			return -1;
		}

		_addCheckSum(rawData);
		return 0;
	}
	
	PacketType resolveData()
	{
		if (!_verifyPacket(rawData))
			return ERROR_CMD;

		switch(rawData[3]) {
		case TIMEOUT:
			return TIMEOUT;
        
		case ERROR_CMD:
			return ERROR_CMD;
        
		case RC_COMMAND:
			speed = (char)rawData[4];
			steer = (char)rawData[5];
			return RC_COMMAND;
        
		case READ_PATH:
			memcpy(pathName, (char*)(rawData+4), rawData[2]);
			return READ_PATH;
        
		case CLEAR_PATH:
			return CLEAR_PATH;
        
		case SAVE_PATH:
			memcpy(pathName, (char*)(rawData+6), rawData[2]);
			return SAVE_PATH;
        
		case ASK_STATUS:
			return ASK_STATUS;
        
		case START_NAV:
			return START_NAV;
        
		case PAUSE_NAV:
			return PAUSE_NAV;
        
		case RESET:
			return RESET;
        
		default:
			return ERROR_CMD;
		}
	}

protected:
	void _addHeader(unsigned char *packet)
	{
		packet[0] = 0xF0;
		packet[1] = 0xFA;
	}

	unsigned char _addCheckSum(unsigned char *packet)
	{
		unsigned char N = packet[2];
		unsigned char sum = 0;
		for (int i=0; i<(N+4); i++)
			sum += packet[i];
		packet[N+4] = sum;
		return sum;
	}

	bool _verifyCheckSum(unsigned char *packet)
	{
		unsigned char N = packet[2];
		unsigned char sum = 0;
		for (int i=0; i<(N+4); i++)
			sum += packet[i];
		return (packet[N+4] == sum);
	}
	
	bool _verifyPacket(unsigned char *packet)
	{
		if (packet[0] != 0xF0 || packet[1] != 0xFA)
			return false;
		return _verifyCheckSum(packet);
	}

	bool _verifyPacket(unsigned char *packet, PacketType type)
	{
		if (_verifyPacket(packet))
			return (packet[3] == type);
		return false;
	}
	
};

};	

#endif
