#include "drive.h"

YT::Drive::Drive()
{
	this->_init();
}

YT::Drive::Drive(struct ControlParameter steer, struct ControlParameter speed)
{
	this->_init();
	_steerPara = steer;
	_speedPara = speed;
}

YT::Drive::~Drive()
{
	setSpeed(0);
	setSteer(0);
}

void YT::Drive::setSteerParameter(struct ControlParameter para)
{
	if (para.range < 0)
		return;
	else if (para.deadzone < 0)
		return;
	_steerPara = para;
}

void YT::Drive::setSpeedParameter(struct ControlParameter para)
{
	if (para.range < 0)
		return;
	else if (para.deadzone < 0)
		return;
	_speedPara = para;
}

YT::ControlParameter YT::Drive::getSteerParameter()
{
	return _steerPara;
}

YT::ControlParameter YT::Drive::getSpeedParameter()
{
	return _speedPara;
}

void YT::Drive::setSteer(int value)
{
	int ret = _calculateCommand(-value, _steerPara);
	_steer.setDutyNs(ret);
}

void YT::Drive::setSpeed(int value)
{
	int ret = _calculateCommand(value, _speedPara);
	if (ret > 0) {
		_speed.setDutyNs(ret);
		_ina.value(1);
		_inb.value(0);
	} else if (ret < 0) {
		_speed.setDutyNs(-ret);
		_ina.value(0);
		_inb.value(1);
	} else {
		_ina.value(0);
		_inb.value(0);
	}
}

void YT::Drive::_init(void)
{
	_steer.open("1A", 50);
	_speed.open("2A", 1000);
	_steer.setDutyNs(0);
	_speed.setDutyNs(0);

	_ina.open(47, 1);

	_inb.open(27, 1);

	_steerPara.zero_pos = 1500000;
	_steerPara.range = 300000;
	_steerPara.deadzone = 10;

	_speedPara.zero_pos = 0;
	_speedPara.range = 300000;
	_speedPara.deadzone = 10;
}

inline int YT::Drive::_calculateCommand(int value, struct ControlParameter para)
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
