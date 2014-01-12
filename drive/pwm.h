#ifndef PWM_H_
#define PWM_H_ 1

#include<stdio.h>
#include<string.h>
#include<errno.h>
#include<stdlib.h>
#include<fcntl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<unistd.h>
#include<limits.h>

#define PWM_PATH "/sys/devices/ocp.3/"

namespace BBB {

class PWM {
public:
	PWM();
	PWM(const char *name, const int freq);
	~PWM();
	void open(const char *name, const int freq);
	void setDutyNs(const int value);

private:
	int _fdNS;
	char _pwmName[40];
};

};
#endif
