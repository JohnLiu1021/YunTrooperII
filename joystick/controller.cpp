#include "controller.h"

Controller::Controller(){};

Controller::Controller(const char *path, IOInterface itf)
{
	openController(path, itf);
}

int Controller::openController(const char *path, IOInterface)
{
	if ((_fd = open(path, O_RDONLY)) < 0) {
		return -1;
	}

	/* Acquiring information */
	ioctl(_fd, JSIOCGVERSION, &_version);
	ioctl(_fd, JSIOCGAXES, &_axes);
	ioctl(_fd, JSIOCGBUTTONS, &_buttons);
	ioctl(_fd, JSIOCGNAME(128), _name);
	ioctl(_fd, JSIOCGAXMAP, _axmap);
	ioctl(_fd, JSIOCGBTNMAP, _btnmap);

	/* Set default interface */
	setInterface(NONBLOCK);

	return 0;
}

int Controller::setInterface(IOInterface itf)
{
	_itf = itf;
	if (itf == NONBLOCK) {
		fcntl(_fd, F_SETFL, O_NONBLOCK);
	} else {
		fcntl(_fd, F_SETFL, 0);
	}
	return 0;
}

int Controller::readButtons(Buttons *btns)
{
	btns->axisNumber = _axes;
	btns->buttonNumber = _buttons;
	struct js_event js;

	switch(_itf) {
	case EVENT:
		if (read(_fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
			return -1;
		}
		switch(js.type & ~JS_EVENT_INIT) {
		case JS_EVENT_BUTTON:
			btns->button[js.number] = js.value;
			break;
		case JS_EVENT_AXIS:
			btns->axis[js.number] = js.value;
			break;
		}
		break;

	case NONBLOCK:
		while(read(_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {
			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				btns->button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				btns->axis[js.number] = js.value;
				break;
			}
		}
		if (errno != EAGAIN) {
			return -1;
		}
		break;

	case SELECT:
		struct timeval tv;
		fd_set set;
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		FD_ZERO(&set);
		FD_SET(_fd, &set);

		if (select(_fd+1, &set, NULL, NULL, &tv)) {
			if (read(_fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
				return -1;
			}
			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				btns->button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				btns->axis[js.number] = js.value;
				break;
			}
		}
		break;
	
	default:
		return -1;
	}
	return 0;
}
