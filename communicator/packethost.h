#ifndef YT_PACKETHOST_H
#define YT_PACKETHOST_H 1

#include "packet.h"

namespace YT {

class PacketHost : public Packet
{
public:
	virtual int packingData()
	{
		addHeader();
		switch(field.packetType) {
		case RC_COMMAND:
			addDataNumber(2);
			addPacketType(RC_COMMAND);
			rawData[4] = (unsigned char) field.speed;
			rawData[5] = (unsigned char) field.steer;
			break;

		case READ_PATH:
			addDataNumber(strlen(field.pathName)+1);
			addPacketType(RC_COMMAND);
			memcpy(rawData+4, field.pathName, strlen(field.pathName)+1);
			break;

		case CLEAR_PATH:
			addDataNumber(0);
			addPacketType(CLEAR_PATH);
			break;

		case SAVE_PATH:
			addDataNumber(strlen(field.pathName)+1);
			addPacketType(SAVE_PATH);
			memcpy(rawData+4, field.pathName, strlen(field.pathName)+1);
			break;

		case ASK_STATUS:
			addDataNumber(0);
			addPacketType(ASK_STATUS);
			break;

		case START_NAV:
			addDataNumber(0);
			addPacketType(START_NAV);
			break;

		case PAUSE_NAV:
			addDataNumber(0);
			addPacketType(PAUSE_NAV);
			break;

		case RESET:
			addDataNumber(0);
			addPacketType(RESET);
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

		case ACK_NOK:
			field.packetType = ACK_NOK;
			break;
        
		case ACK_READ_PATH:
			field.packetType = ACK_READ_PATH;
			field.pathPointNumber = rawData[4];
			break;
        
		case ACK_SAVE_PATH:
			field.packetType = ACK_SAVE_PATH;
			field.pathPointNumber = rawData[4];
			break;
        
		case ACK_STATUS:
			field.packetType = ACK_STATUS;
			memcpy(&(field.latitude), rawData+4, 8);
			memcpy(&(field.longitude), rawData+12, 8);
			memcpy(&(field.yaw), rawData+20, 8);
			field.statusBit = rawData[28];
			break;
        
		case ACK_START_NAV:
			field.packetType = ACK_START_NAV;
			field.pathPointNumber = rawData[4];
			break;
        
		case ACK_PAUSE_NAV:
			field.packetType = ACK_PAUSE_NAV;
			field.pathPointNumber = rawData[4];
			break;
        
		case ACK_CLEAR_PATH:
			field.packetType = ACK_CLEAR_PATH;
			break;
        
		case ACK_RESET:
			field.packetType = ACK_RESET;
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



