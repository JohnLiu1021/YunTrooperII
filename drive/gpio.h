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
	GPIO();
	GPIO(int gpioNumber);
	GPIO(int gpioNumber, int direction);

	int open(int gpioNumber, int dir);
	int setDirection(int dir);
	int setEdge(const char *edge);
	int value();
	int value(int level);

protected:
	int getFd();
	int exportGPIO();
	int unexportGPIO();

private:
	int _direction;
	int _gpioNumber;
	char _valuePath[40];
	int _valueFd;

};

};
#endif
