#ifndef GPIO_H
#define GPIO_H 1

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#define EXPORT_PATH	"/sys/class/gpio/export"
#define UNEXPORT_PATH	"/sys/class/gpio/unexport"

namespace BBB {

class GPIO {
public:
	GPIO(){};
	GPIO(int gpioNumber)
	{
		this->open(gpioNumber);
	}

	int open(int gpioNumber)
	{
		_direction = 0;
		_gpioNumber = gpioNumber;
		_valueFd = -1;
		high[0] = '1';
		high[1] = 0;
		low[0] = '0';
		low[1] = 0;
		_valuePath = NULL;
		this->exportGPIO();
		this->setDirection(1);
	}

	~GPIO()
	{
		if (_direction == 0) {
			free(_valuePath);
		}

		unexportGPIO();
	}
	
	int setDirection(int dir)
	{
		char path[PATH_MAX];
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
			close(dir_fd);
			_direction = 1;
        
		} else { // Input
			wr = write(dir_fd, "in", 2);
			if (wr == -1) {
				perror("write : direction fd");
				exit(EXIT_FAILURE);
			}
			close(dir_fd);
		
			this->setEdge("both");
			_direction = 0;
		}
		sprintf(path, "/sys/class/gpio/gpio%d/value", _gpioNumber);
		if (_direction == 0) {
			if (!_valuePath)
				_valuePath = (char*)calloc(sizeof(char), strlen(path));
			strcpy(_valuePath, path);
		}

		_valueFd = ::open(path, O_RDWR);
		if (_valueFd == -1) {
			perror("open : value fd");
			exit(EXIT_FAILURE);
		}
		
		if (_direction)
			this->value(0);
        
		return 0;
	}

	int setEdge(const char *edge)
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
		close(edge_fd);
		return 0;
	}

	int value()
	{
		char buf[2];
        
		close(_valueFd);
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
			perror("read : input state");
			exit(EXIT_FAILURE);
		}
	}

	int value(int level)
	{
		if (_direction == 0)
			return -1;
		int wr;
        
		if (level)
			wr = write(_valueFd, high, 2);
		else
			wr = write(_valueFd, low, 2);
        	
		return wr;
	}

protected:
	int getFd()
	{
		return _valueFd;
	}

	int exportGPIO()
	{
		char path[PATH_MAX];
		char GPIONum[4];
		int export_fd;
		int wr;
        
		// Export the pin
		export_fd = ::open(EXPORT_PATH, O_WRONLY);
		if (export_fd == -1) {
			perror("open : export fd");
			exit(EXIT_FAILURE);
		}
		sprintf(GPIONum, "%d", _gpioNumber);

Export:		
		if ((wr = write(export_fd, GPIONum, strlen(GPIONum))) == -1) {
			if (errno == EBUSY) {
				this->unexportGPIO();
				goto Export;
			} else {
				perror("write : export fd");
				exit(EXIT_FAILURE);
			}
		}
		close(export_fd);
	}

	int unexportGPIO()
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
        
		close(unexport_fd);
		return 0;
	}

private:
	int _direction;
	int _gpioNumber;
	char high[2];
	char low[2];
	char *_valuePath;
	int _valueFd;

};

};
#endif
