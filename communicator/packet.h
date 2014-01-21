/*
   Pure virtual class of packet type.
*/
#ifndef YT_PACKET_H
#define YT_PACKET_H 1

#include <string.h> // memcpy
#define PACKET_SIZE 30

namespace YT {

enum HeaderData{
	HEADER1 = 0xF0,
	HEADER2 = 0xFA
};

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
	RC_COMMAND	= 0x02,
	READ_PATH	= 0x06,
	CLEAR_PATH	= 0x16,
	SAVE_PATH	= 0x26,
	ASK_STATUS	= 0x1A,
	START_NAV	= 0xBB,
	PAUSE_NAV	= 0xB1,
	RESET		= 0xDD,
	
	/* Error Type */
	ERROR_CMD	= 0xFF,
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
		char pathName[20];
		unsigned char statusBit;
		unsigned char pathPointNumber;
		double latitude;
		double longitude;
		double yaw;
		PacketType packetType;
	} field;
	
	/* Raw data array */
	unsigned char rawData[PACKET_SIZE];

protected:
	/* Functions dealing with packet format */
	inline void addHeader()
	{
		rawData[0] = HEADER1;
		rawData[1] = HEADER2;
	}

	inline void addDataNumber(const int number)
	{
		rawData[2] = (unsigned char) number;
	}

	inline int getDataNumber()
	{
		return rawData[2];
	}

	inline void addPacketType(PacketType type)
	{
		rawData[3] = (unsigned char) type;
	}

	inline unsigned char addCheckSum()
	{
		unsigned char N = rawData[2];
		unsigned char sum = 0;
		for (int i=0; i<(N+4); i++)
			sum += rawData[i];
		rawData[N+4] = sum;
		return sum;
	}

	inline bool verifyCheckSum()
	{
		unsigned char N = rawData[2];
		unsigned char sum = 0;
		for (int i=0; i<(N+4); i++)
			sum += rawData[i];
		return (rawData[N+4] == sum);
	}
	
	inline unsigned char verifyPacket()
	{
		if (rawData[0] != HEADER1 || rawData[1] != HEADER2)
			return ERROR_CMD;

		if (verifyCheckSum()) 
			return rawData[3];
		else 
			return ERROR_CMD;
	}

	inline bool verifyPacket(PacketType type)
	{
		if (verifyPacket())
			return (rawData[3] == type);
		return false;
	}
};

};	

#endif
