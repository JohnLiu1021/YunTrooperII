#include "pwm.h"

BBB::PWM::PWM(){};
BBB::PWM::PWM(const char *name, const int freq = 500)
{
	this->open(name, freq);
}

BBB::PWM::~PWM()
{
	close(_fdNS);
	char pathBuffer[PATH_MAX];
	strcpy(pathBuffer, PWM_PATH);
	strcat(pathBuffer, _pwmName);
	strcat(pathBuffer, "run");

	int fd = ::open(pathBuffer, O_RDWR);
	if (fd == -1) {
		printf("pathBuffer : %s\n", pathBuffer);
		perror("destructor : open : run fd");
		exit(EXIT_FAILURE);
	}

	write(fd, "0", 1);
	close(fd);
}

void BBB::PWM::open(const char *name, const int freq = 500)
{
	char pathbuffer[255];
	char PwmPathConst[255];
	char inputBuffer[10];
	size_t wr;
	int fd;

	strcpy(PwmPathConst, PWM_PATH);
	if (!strcmp(name, "1A") || !strcmp(name, "1a")) {
		strcpy(_pwmName, "pwm_test_P9_14.16/");
	}
	else if (!strcmp(name, "2A") || !strcmp(name, "2a")) {
		strcpy(_pwmName, "pwm_test_P8_19.17/");
	}
	strcat(PwmPathConst, _pwmName);

	strcpy(pathbuffer, PwmPathConst);
	strcat(pathbuffer, "period");
	
	fd = ::open(pathbuffer, O_RDWR);
	if (fd == -1) {
		perror("open : period fd");
		printf("The path is : %s\n", pathbuffer);
		exit(EXIT_FAILURE);
	}
	int period = 1000000000 / freq;
	sprintf(inputBuffer, "%d", period);
	
	wr = write(fd, inputBuffer, strlen(inputBuffer));
	if (wr < strlen(inputBuffer)) {
		perror("write : period fd");
		exit(EXIT_FAILURE);
	}
	close(fd);

	strcpy(pathbuffer, PwmPathConst);
	strcat(pathbuffer, "polarity");
	
	fd = ::open(pathbuffer, O_RDWR);
	if (fd == -1) {
		perror("open : polarity fd");
		exit(EXIT_FAILURE);
	}
	wr = write(fd, "0", 1);
	close(fd);

	strcpy(pathbuffer, PwmPathConst);
	strcat(pathbuffer, "run");
	
	fd = ::open(pathbuffer, O_RDWR);
	if (fd == -1) {
		perror("open : run fd");
		exit(EXIT_FAILURE);
	}
	wr = write(fd, "1", 1);
	close(fd);
	
	strcpy(pathbuffer, PwmPathConst);
	strcat(pathbuffer, "duty");
	_fdNS = ::open(pathbuffer, O_RDWR);
	if (_fdNS == -1) {
		perror("open : duty");
		exit(EXIT_FAILURE);
	}
}


void BBB::PWM::setDutyNs(const int value)
{
	char inputBuffer[10];
	sprintf(inputBuffer, "%d", value);
	write(_fdNS, inputBuffer, strlen(inputBuffer));
	return;
}
