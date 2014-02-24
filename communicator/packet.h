/*
   Pure virtual class of packet type.
*/
#ifndef YT_PACKET_H
#define YT_PACKET_H 1

#include <string.h> // memcpy
#define PACKET_SIZE 40

namespace YT {

enum HeaderData{
	HEADER1 = 0xF0,
	HEADER2 = 0xFA
};

enum PacketType{
	/* Clent to Host type */

	/* 
	   Return the number of remaining path point number
	   Length = 1
	   packet[4] = Number of path point
	*/
	ACK_OK 		= 0x94,

	/* 
	   ACK_NOK : Invalid requirement
	   Length = 0
	*/
	ACK_NOK 	= 0x99,

	/* 
	   Return the status of the sensor
	   Length = 24 
	   packet[4..11] = latitude
	   packet[12..19] = longitude
	   packet[20..27] = yaw
	   packet[28] = Sensor status bit
	   packet[29] = Totoal path point number
	*/
	ACK_STATUS	= 0x93,

	/* 
	   Sending the latitude and longitude of each path point back to the host
	   Length = 2 + 8 + 8 = 18
	   packet[4] = Total number of path point
	   packet[5] = number of the path point
	   packet[6..13] = latitude of the path point
	   packet[14..21] = longitude of the path point
	*/
	ACK_SHOW_PATH	= 0x97,

	/* Timeout */
	TIMEOUT		= 0,

	/* From Host to Client Type */

	/* 
	   Remote control command
	   Length = 2
	   packet[4] = speed command (from -100 to 100)
	   packet[5] = steer command (from -100 to 100)
	*/
	RC_COMMAND	= 0x02,

	/* 
	   Ask Yun Trooper II to read the path from a specific file name
	   Length = depend on the length of file name 
	   packet[4..N] = file name
	*/
	READ_PATH	= 0x06,

	/* 
	   Clear all path points
	   Length = 0
	*/
	CLEAR_PATH	= 0x16,

	/* 
	   Save current path points to a specific file
	   Legnth = depend on the length of file name
	   packet[4..N] = file name
	*/
	SAVE_PATH	= 0x26,

	/* 
	   Asking for status
	   Length = 0
	*/
	ASK_STATUS	= 0x1A,

	/* 
	   Toggle navigation
	   Length = 0
	*/
	TOGGLE_NAV	= 0x1B,

	/* 
	   Reset Yun Trooper II
	   Length = 0
	*/
	RESET		= 0xAA,

	/* 
	   Show all path point
	   Length = 0
	*/
	SHOW_PATH	= 0xBB,

	/* 
	   Adding path point from the command
	   Length = 16
	   packet[4..11] = latitude
	   packet[12..19] = longitude
	*/
	ADD_PATH	= 0xCC,

	/* 
	   Remove specific path point number from the path point set
	   Length = 1
	   packet[4] = number of path point
	*/
	REMOVE_PATH	= 0xDD,
	
	/* Error Type */
	ERROR_CMD	= 0xFF
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
		int totalPathPointNumber;
		int pathPointNumber;
		unsigned char statusBit;
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
