#include "communicator.h"

YT::Communicator::Communicator()
{
	pthread_mutex_init(&(_sharedData.mutex), NULL);

	/* Initialize raw data with packet TIMEOUT */
	_sharedData.rawData[0] = 0xF0;
	_sharedData.rawData[1] = 0xFA;
	_sharedData.rawData[2] = 0;
	_sharedData.rawData[3] = YT::TIMEOUT;
	_sharedData.rawData[4] = 0xEA;

	_sharedData.portFd = 0;
	_sharedData.Flag_isUpdate = false;
}

YT::Communicator::~Communicator()
{
	close();
}

int YT::Communicator::open(const char *name, const int baudRate)
{
	int baud;
	switch(baudRate) {
		case 2400:
			baud = B2400;
			break;
		case 4800:
			baud = B4800;
			break;
		case 9600:
			baud = B9600;
			break;
		case 19200:
			baud = B19200;
			break;
		case 38400:
			baud = B38400;
			break;
		case 57600:
			baud = B57600;
			break;
		case 115200:
			baud = B115200;
			break;
		default:
			printf("Wrong baud rate format!\n");
			exit(EXIT_FAILURE);
	}
	_sharedData.portFd = ::open(name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (_sharedData.portFd == -1) {
		fprintf(stderr, "%s", name);
		perror(" : open");
		exit(EXIT_FAILURE);
	}
	fcntl(_sharedData.portFd, F_SETFL, 0);

	struct termios options;
	tcgetattr(_sharedData.portFd, &options);
	cfsetispeed(&options, baud);
	cfsetospeed(&options, baud);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_lflag = 0;
	options.c_cc[VTIME]	= 1;
	options.c_cc[VMIN]	= 1;
	options.c_iflag |= IGNPAR;
	tcsetattr(_sharedData.portFd, TCSANOW, &options);
	tcflush(_sharedData.portFd, TCIOFLUSH);

	if (pthread_create(&_thread, NULL, YT::Communicator::_handle, (void*)&_sharedData) < 0) {
		perror("thread create : ");
		exit(EXIT_FAILURE);
	}

	return 0;
}

int YT::Communicator::close()
{
	if (_sharedData.portFd)
		::close(_sharedData.portFd);
	pthread_cancel(_thread);
	return 0;
}

int YT::Communicator::flushData()
{
	tcflush(_sharedData.portFd, TCIOFLUSH);
	return 0;
}

YT::PacketType YT::Communicator::readCommand(YT::Packet *packet)
{
	pthread_mutex_lock(&(_sharedData.mutex));
	if (!_sharedData.Flag_isUpdate) {
		pthread_mutex_unlock(&(_sharedData.mutex));
		return YT::TIMEOUT;
	}

	memcpy(packet->rawData, _sharedData.rawData, PACKET_SIZE);
	_sharedData.Flag_isUpdate = false;
	pthread_mutex_unlock(&(_sharedData.mutex));
	return packet->resolveData();
}

int YT::Communicator::sendCommand(YT::Packet *packet)
{
	packet->packingData();
	printf("Sending command!\n");
	tcflush(_sharedData.portFd, TCOFLUSH);
	int wr = write(_sharedData.portFd, packet->rawData, (packet->rawData[2] + 5));
	if (wr < 0) {
		perror("write : portFd");
		return -1;
	}
	return 0;
}

void *YT::Communicator::_handle(void *ptr)
{
	struct _SharedData *s = (struct _SharedData *)ptr;
	pthread_cleanup_push(YT::Communicator::_cleanup_function, ptr);

	byte localBuffer[PACKET_SIZE];

	Ring::RingBuffer ring;
	Ring::initialize(&ring, 30);

	unsigned char header[2] = {0xF0, 0xFA};

	while(1) {
		int rd, ret;
		rd = read(s->portFd, localBuffer, PACKET_SIZE);

		if (rd > 0) {
			ret = Ring::write(&ring, localBuffer, rd);
			if (ret < 0) {
				fprintf(stderr, "RingBuffer : Not enough space, clear all data\n");
				Ring::clear(&ring);
			}
			
			if ((ret = Ring::find(&ring, header, 2, Ring::FROM_TAIL)) >= 0) {
				ring.tail = ret;
				if (Ring::size(&ring) < 5) {
					continue;
				}
				
				int packetSize = ring.buffer[ring.tail + 2] + 5;
				if (Ring::size(&ring) < packetSize) {
					continue;
				}
				pthread_mutex_lock(&(s->mutex));
				Ring::read(&ring, s->rawData, packetSize);
				s->Flag_isUpdate = true;
				pthread_mutex_unlock(&(s->mutex));
			}
		} else if (rd == 0) {
			printf("Timeout\n");
		} else {
			perror("read : portFd");
		}
		
	}
	pthread_cleanup_pop(0);
}

void YT::Communicator::_cleanup_function(void *ptr)
{
	struct _SharedData *s = (struct _SharedData *)ptr;
	pthread_mutex_unlock(&(s->mutex));
}
