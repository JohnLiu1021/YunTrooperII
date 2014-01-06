#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H 1

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <pthread.h>
#include <cstddef>

#include "packet.h"

typedef unsigned char byte;

namespace YT {

class Communicator {
private:
	pthread_t _thread;
	static void *_handle(void *);
	static void _cleanup_function(void *);

	struct _SharedData {
		int portFd;			// fd of serial port
		pthread_mutex_t mutex;		// mutex lock for this shared struct
		byte rawData[PACKET_SIZE];	// storing the data
		bool Flag_isUpdate;		// indicating whether the data has read by xbee
	};
	struct _SharedData _sharedData;
	
public:
	Communicator();
	~Communicator();
	int open(const char *, const int);
	int close();
	int flushData();
	YT::PacketType readCommand(YT::Packet *);
	int sendCommand(YT::Packet *);
};

};
#endif
