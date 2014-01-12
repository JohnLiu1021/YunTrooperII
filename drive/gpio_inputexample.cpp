#include <stdio.h>
#include "gpio.h"

int main(void)
{
	BBB::GPIO btn1(66, 0);
	btn1.setEdge("rising");

	while(1) {
		if (btn1.value() == 1)
			printf("Pressed!\n");
		usleep(100000);
	}
	return 0;
}
