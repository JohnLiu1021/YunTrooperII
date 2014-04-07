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
			addPacketType(READ_PATH);
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

		case TOGGLE_NAV:
			addDataNumber(0);
			addPacketType(TOGGLE_NAV);
			break;

		case RESET:
			addDataNumber(0);
			addPacketType(RESET);
			break;

		case SKIP:
			addDataNumber(0);
			addPacketType(SKIP);
			break;
			
		case SHOW_PATH:
			addDataNumber(0);
			addPacketType(SHOW_PATH);
			break;

		case ADD_PATH:
			addDataNumber(16);
			addPacketType(ADD_PATH);
			memcpy(rawData+4, &(field.latitude), 8);
			memcpy(rawData+12, &(field.longitude), 8);
			break;

		case REMOVE_PATH:
			addDataNumber(1);
			addPacketType(REMOVE_PATH);
			rawData[4] = (unsigned char)field.pathPointNumber;
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

		case ACK_OK:
			field.packetType = ACK_OK;
			field.statusBit = rawData[4];
			field.pathPointNumber = rawData[5];
			field.totalPathPointNumber = rawData[6];
			break;
        
		case ACK_NOK:
			field.packetType = ACK_NOK;
			break;
        
		case ERROR_CMD:
			field.packetType = ERROR_CMD;
			break;

		case TIMEOUT:
			field.packetType = TIMEOUT;
			break;

		case ACK_STATUS:
			field.packetType = ACK_STATUS;
			memcpy(&(field.latitude), rawData+4, 8);
			memcpy(&(field.longitude), rawData+12, 8);
			memcpy(&(field.yaw), rawData+20, 8);
			field.statusBit = rawData[28];
			field.pathPointNumber = rawData[29];
			field.totalPathPointNumber = rawData[30];
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



