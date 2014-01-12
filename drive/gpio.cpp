#include "gpio.h"

BBB::GPIO::GPIO() {}

BBB::GPIO::GPIO(const int gpioNumber)
{
	this->open(gpioNumber, 1);
}

BBB::GPIO::GPIO(const int gpioNumber, const int direction)
{
	this->open(gpioNumber, direction);
}

int BBB::GPIO::open(const int gpioNumber, const int dir)
{
	_gpioNumber = gpioNumber;
	this->exportGPIO();
	this->setDirection(dir);
	return 0;
}

int BBB::GPIO::setDirection(const int dir)
{
	_direction = dir;
	char path[255];
	int dir_fd;
	int wr;
	sprintf(path, "/sys/class/gpio/gpio%d/direction", _gpioNumber);
	
	dir_fd = ::open(path, O_WRONLY);
	if (dir_fd == -1) {
		perror("open : direction fd");
		exit(EXIT_FAILURE);
	}

	if (dir) { // Output
		wr = write(dir_fd, "out", 3);
		if (wr == -1) {
			perror("write : direction fd");
			exit(EXIT_FAILURE);
		}

	} else { // Input
		wr = write(dir_fd, "in", 2);
		if (wr == -1) {
			perror("write : direction fd");
			exit(EXIT_FAILURE);
		}
		this->setEdge("rising");
	}
	::close(dir_fd);

	sprintf(_valuePath, "/sys/class/gpio/gpio%d/value", _gpioNumber);
	_valueFd = ::open(_valuePath, O_RDWR);

	if (_valueFd == -1) {
		perror("open : value fd");
		exit(EXIT_FAILURE);
	}
	
	if (_direction)
		this->value(0);

	return 0;
}

int BBB::GPIO::setEdge(const char *edge)
{
	char path[PATH_MAX];
	sprintf(path, "/sys/class/gpio/gpio%d/edge", _gpioNumber);
	int edge_fd = ::open(path, O_WRONLY);
	if (edge_fd == -1) {
		perror("open : edge fd");
		exit(EXIT_FAILURE);
	}

	int wr = write(edge_fd, edge, strlen(edge));
	if (wr == -1) {
		perror("write : edge fd");
		exit(EXIT_FAILURE);
	}
	::close(edge_fd);
	return 0;
}

int BBB::GPIO::value()
{
	char buf[2];

	::close(_valueFd);
	_valueFd = ::open(_valuePath, O_RDWR);
	int rd = read(_valueFd, buf, 2);
	if (rd > 0) {
		switch (buf[0]) {
		case '0':
			return 0;
		case '1':
			return 1;
		default:
			return -1;
		}
	} else {
		perror("read: input state");
		exit(EXIT_FAILURE);
	}
}

int BBB::GPIO::value(const int level)
{
	if (_direction == 0)
		return -1;
	int wr;
	if (level)
		wr = write(_valueFd, "1", 1);
	else
		wr = write(_valueFd, "0", 1);
	
	return wr;
}

int BBB::GPIO::getFd()
{
	return _valueFd;
}

int BBB::GPIO::exportGPIO()
{
	char GPIONum[4];
	int export_fd;

	// Export the pin
	export_fd = ::open(EXPORT_PATH, O_WRONLY);
	if (export_fd == -1) {
		perror("open : export fd");
		exit(EXIT_FAILURE);
	}
	sprintf(GPIONum, "%d", _gpioNumber);

	while(write(export_fd, GPIONum, strlen(GPIONum)) == -1) {
		if (errno == EBUSY) {
			this->unexportGPIO();
		} else {
			perror("write : export fd");
			exit(EXIT_FAILURE);
		}
	}
	::close(export_fd);
	return 0;
}

int BBB::GPIO::unexportGPIO()
{
	int unexport_fd = ::open(UNEXPORT_PATH, O_WRONLY);
	if (unexport_fd == -1) {
		perror("open : export fd");
		exit(EXIT_FAILURE);
	}
	int wr;
	char GPIONum[4];
	sprintf(GPIONum, "%d", _gpioNumber);
	
	if ((wr = write(unexport_fd, GPIONum, strlen(GPIONum))) == -1) {
		perror("write : unexport");
		exit(EXIT_FAILURE);
	}

	::close(unexport_fd);
	return 0;
}
