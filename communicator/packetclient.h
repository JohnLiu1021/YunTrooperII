#ifndef YT_PACKETCLIENT_H
#define YT_PACKETCLIENT_H 1

#include "packet.h"

namespace YT {

class PacketClient : public Packet
{
public:
	virtual int packingData()
	{
		addHeader();
		switch(field.packetType) {

		case ACK_OK:
			addDataNumber(3);
			addPacketType(ACK_OK);
			rawData[4] = field.statusBit;
			rawData[5] = field.pathPointNumber;
			rawData[6] = (unsigned char)field.totalPathPointNumber;
			break;
        
		case ACK_NOK:
			addDataNumber(0);
			addPacketType(ACK_NOK);
			break;
        
		case ACK_STATUS:
			addDataNumber(27);
			addPacketType(ACK_STATUS);
			memcpy(rawData+4, &(field.latitude), 8);
			memcpy(rawData+12, &(field.longitude), 8);
			memcpy(rawData+20, &(field.yaw), 8);
			rawData[28] = field.statusBit;
			rawData[29] = (unsigned char)field.pathPointNumber;
			rawData[30] = (unsigned char)field.totalPathPointNumber;
			break;
        
		case ACK_SHOW_PATH:
			addDataNumber(18);
			addPacketType(ACK_SHOW_PATH);
			rawData[4] = (unsigned char)field.totalPathPointNumber;
			rawData[5] = field.pathPointNumber;
			memcpy(rawData+6, &(field.latitude), 8);
			memcpy(rawData+14, &(field.longitude), 8);
			break;
        
		default:
			return -1;
		}

		addCheckSum();
		return 0;
	}

	virtual PacketType resolveData()
	{
		switch(verifyPacket()) {
		case ERROR_CMD:
			field.packetType = ERROR_CMD;
			break;
        
		case TIMEOUT:
			field.packetType = TIMEOUT;
			break;
        
		case RC_COMMAND:
			field.packetType = RC_COMMAND;
			field.speed = (signed char)rawData[4];
			field.steer = (signed char)rawData[5];
			break;
        
		case READ_PATH:
			field.packetType = READ_PATH;
			memcpy(field.pathName, (char*)(rawData+4), getDataNumber());
			break;
        
		case CLEAR_PATH:
			field.packetType = CLEAR_PATH;
			break;
        
		case SAVE_PATH:
			field.packetType = SAVE_PATH;
			memcpy(field.pathName, (char*)(rawData+4), getDataNumber());
			break;
        
		case ASK_STATUS:
			field.packetType = ASK_STATUS;
			break;
        
		case TOGGLE_NAV:
			field.packetType = TOGGLE_NAV;
			break;
        
		case RESET:
			field.packetType = RESET;
			break;

		case SKIP:
			field.packetType = SKIP;
			break;

		case SHOW_PATH:
			field.packetType = SHOW_PATH;
			break;

		case ADD_PATH:
			field.packetType = ADD_PATH;
			memcpy(&(field.latitude), rawData+4, 8);
			memcpy(&(field.longitude), rawData+12, 8);
			break;

		case REMOVE_PATH:
			field.packetType = REMOVE_PATH;
			field.pathPointNumber = rawData[4];
			break;
        
		default:
			field.packetType = ERROR_CMD;
			break;
		}
		return field.packetType;
	}
};

};

#endif
