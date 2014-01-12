#include "gpio.h"

int main(void)
{
	BBB::GPIO led1(69, 1);
	while(1) {
		led1.value(1);
		usleep(500000);
		led1.value(0);
		usleep(500000);
	};
	return 0;
}
