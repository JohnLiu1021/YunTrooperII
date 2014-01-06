#ifndef DRIVE_H
#define DRIVE_H 1

#include "pwm.h"
#include "gpio.h"

/* 
   Controlling the motion of YunTropperII.
   Both speed and steering commands are limited from 100 to -100.
   The transformation between command and real control value 
   will be done automatcally by taking the ConstrolParameter
   structure into account and calculating the relation.
*/

struct ControlParameter {
	int zero_pos;
	int range;
	int deadzone; // In percentage
};

class Drive {
public:
	Drive();
	Drive(struct ControlParameter steer, struct ControlParameter speed);

	void setSteerParameter(struct ControlParameter);
	void setSpeedParameter(struct ControlParameter);

	struct ControlParameter getSteerParameter();
	struct ControlParameter getSpeedParameter();

	void setSteer(int);
	void setSpeed(int);

private:
	struct ControlParameter _steerPara;
	struct ControlParameter _speedPara;
	BBB::PWM _steer;
	BBB::PWM _speed;
	BBB::GPIO _ina;
	BBB::GPIO _inb;

	void _init(void);
	inline int _calculateCommand(int, struct ControlParameter);
};

#endif
