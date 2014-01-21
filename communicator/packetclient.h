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
		case ACK_NOK:
			addDataNumber(0);
			addPacketType(ACK_NOK);
			break;
        
		case ACK_READ_PATH:
			addDataNumber(1);
			addPacketType(ACK_READ_PATH);
			rawData[4] = field.pathPointNumber;
			break;
        
		case ACK_SAVE_PATH:
			addDataNumber(1);
			addPacketType(ACK_SAVE_PATH);
			rawData[4] = field.pathPointNumber;
			break;
        
		case ACK_STATUS:
			addDataNumber(25);
			addPacketType(ACK_STATUS);
			memcpy(rawData+4, &(field.latitude), 8);
			memcpy(rawData+12, &(field.longitude), 8);
			memcpy(rawData+20, &(field.yaw), 8);
			rawData[28] = field.statusBit;
			break;
        
		case ACK_START_NAV:
			addDataNumber(1);
			addPacketType(ACK_START_NAV);
			rawData[4] = field.pathPointNumber;
			break;
        
		case ACK_PAUSE_NAV:
			addDataNumber(1);
			addPacketType(ACK_PAUSE_NAV);
			rawData[4] = field.pathPointNumber;
			break;
        
		case ACK_CLEAR_PATH:
			addDataNumber(0);
			addPacketType(ACK_CLEAR_PATH);
			break;
        
		case ACK_RESET:
			addDataNumber(0);
			addPacketType(ACK_RESET);
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
			memcpy(field.pathName, (char*)(rawData+6), getDataNumber());
			break;
        
		case ASK_STATUS:
			field.packetType = ASK_STATUS;
			break;
        
		case START_NAV:
			field.packetType = START_NAV;
			break;
        
		case PAUSE_NAV:
			field.packetType = PAUSE_NAV;
			break;
        
		case RESET:
			field.packetType = RESET;
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
