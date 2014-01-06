#include "drive.h"

Drive::Drive()
{
	this->_init();
}

Drive::Drive(struct ControlParameter steer, struct ControlParameter speed)
{
	this->_init();
	_steerPara = steer;
	_speedPara = speed;
}

void Drive::setSteerParameter(struct ControlParameter para)
{
	if (para.range < 0)
		return;
	else if (para.deadzone < 0)
		return;
	_steerPara = para;
}

void Drive::setSpeedParameter(struct ControlParameter para)
{
	if (para.range < 0)
		return;
	else if (para.deadzone < 0)
		return;
	_speedPara = para;
}

struct ControlParameter Drive::getSteerParameter()
{
	return _steerPara;
}

struct ControlParameter Drive::getSpeedParameter()
{
	return _speedPara;
}

void Drive::setSteer(int value)
{
	int ret = _calculateCommand(value, _steerPara);
	_steer.setDutyNs(ret);
}

void Drive::setSpeed(int value)
{
	int ret = _calculateCommand(value, _speedPara);
	if (value > 0) {
		_ina.value(1);
		_inb.value(0);
		_speed.setDutyNs(ret);
	} else if (value < 0) {
		_ina.value(0);
		_inb.value(1);
		_speed.setDutyNs(-ret);
	} else {
		_ina.value(0);
		_inb.value(0);
	}

}

void inline Drive::_init(void)
{
	printf("OpenPWM\n");
	_steer.open("1A", 500);
	printf("First done\n");
	_speed.open("2A", 1000);
	printf("OpenPWM Done\n");
	printf("setDuty\n");
	_steer.setDutyNs(0);
	_speed.setDutyNs(0);
	printf("setDutyDone\n");

	_ina.open(47);
	_ina.setDirection(1);

	_inb.open(27);
	_ina.setDirection(1);

	_steerPara.zero_pos = 1500000;
	_steerPara.range = 300000;
	_steerPara.deadzone = 2;

	_speedPara.zero_pos = 0;
	_speedPara.range = 300000;
	_speedPara.deadzone = 2;
}

int Drive::_calculateCommand(int value, struct ControlParameter para)
{
	if (value > 100)
		value = 100;
	else if (value < -100)
		value = -100;

	value = (value * para.range) / 100;

	int deadzone_value = para.deadzone * para.range / 100;

	if (value >= deadzone_value || value <= -deadzone_value) {
		return para.zero_pos + value;
	} else {
		return para.zero_pos;
	}
}
