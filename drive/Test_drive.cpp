#include <sys/time.h>
#include <signal.h>
#include <math.h>

#include <vector>

#include "drive.h"

#define PERIOD 10000 // us
#define SIN_FREQ 0.2 // Hz

int quit = 0;

void alarm_handler(int signo){};
void ctrlc_handler(int signo)
{
	quit = 1;
}
	

double *sin_generator(double sampling_freq, double freq, int &size)
{
	double digital_freq = freq / sampling_freq;
	double freq_rad = 2 * M_PI * digital_freq;

	size = 1 / digital_freq;
	printf("Size of sin = %d\n", size);

	double *buffer_in = new double(size);
	
	for(int i=0; i<size; i++) {
		double value = freq_rad * (double)i;
		buffer_in[i] = sin(value);
	}

	return buffer_in;
}

int main(void)
{
	double sampling_freq = 1000000.0 / (double) PERIOD;
	printf("sampling freq = %f\n", sampling_freq);
	int size;
	double *sin_data = sin_generator(sampling_freq, SIN_FREQ, size);
	
	for (int i=0; i<size; i++) {
		sin_data[i] *= 100;
	}

	struct itimerval delay;
	int ret;

	signal(SIGALRM, alarm_handler);
	signal(SIGINT, ctrlc_handler);

	delay.it_value.tv_sec = 0; 
	delay.it_value.tv_usec = PERIOD;
	delay.it_interval.tv_sec = 0;
	delay.it_interval.tv_usec = PERIOD;
	ret = setitimer(ITIMER_REAL, &delay, NULL);
	if (ret) {
		fprintf(stderr, "error!\n");
		return -1;
	}

	printf("declare control\n");
	Drive control;
	printf("Sin wave freq = %d Hz\n", SIN_FREQ);
	
	while (!quit) {
		for (int i=0; i<size; i++) {
			pause();
			if (quit)
				break;

			int control_data = (int)sin_data[i];
			control.setSteer(control_data);
			control.setSpeed(control_data);
			printf("control_data = %d\n", control_data);
		}
	}
	printf("Program Ended!\n");
	control.setSteer(0);
	control.setSpeed(0);
	return 0;
}
