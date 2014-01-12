#ifndef CONTROLLER_H
#define CONTROLLER_H 1

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <linux/input.h>
#include <linux/joystick.h>

enum IOInterface {
	EVENT,
	NONBLOCK,
	SELECT
};

class Buttons {
public:
	short axis[8];
	char button[11];
	int axisNumber;
	int buttonNumber;

	short AxisX() {return axis[0];}
	short AxisY() {return axis[1];}
	short AxisLT() {return axis[2];}
	short AxisZ1() {return axis[3];}
	short AxisZ2() {return axis[4];}
	short AxisRT() {return axis[5];}
	short AxisHatX() {return axis[6];}
	short AxisHatY() {return axis[7];}
	
	char BtnA() {return button[0];}
	char BtnB() {return button[1];}
	char BtnX() {return button[2];}
	char BtnY() {return button[3];}
	char BtnLB() {return button[4];}
	char BtnRB() {return button[5];}
	char BtnBack() {return button[6];}
	char BtnSelect() {return button[7];}
	char BtnXBOX() {return button[8];}
	char BtnThumbL() {return button[9];}
	char BtnThumbR() {return button[10];}
};

class Controller {
public:
	Controller();
	Controller(const char *, IOInterface);
	
	int openController(const char *, IOInterface);
	int setInterface(IOInterface);
	int readButtons(struct Buttons *);
private:
	int _fd;
	IOInterface _itf;
	unsigned char _axes;
	unsigned char _buttons;
	int _version;
	char _name[128];
	uint16_t _btnmap[KEY_MAX - BTN_MISC + 1];
	uint8_t _axmap[ABS_MAX + 1];
};

#endif
