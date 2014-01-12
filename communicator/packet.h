/*
   Pure virtual class of packet type.
*/
#ifndef YT_PACKET_H
#define YT_PACKET_H 1

#include <string.h> // memcpy
#define PACKET_SIZE 30

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

	/* Received Type */
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

class Packet {
public:
	/* Packing data from data field to rawData array */
	virtual int packingData() = 0; 

	/* Resolve data from rawData array to data field */
	virtual PacketType resolveData() = 0;

	struct DataField { 
		char speed;
		char steer;
		char pathName[255];
		double latitude;
		double longitude;
		double yaw;
		int pathPointNumber;
		int statusBit;
		PacketType packetType;
	} field;
	
	/* Raw data array */
	unsigned char rawData[PACKET_SIZE];

	void getHeader(unsigned char *buffer, int *size)
	{
		buffer[0] = 0xF0;
		buffer[1] = 0xFA;
		*size = 2;
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
