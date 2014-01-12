#include "controller.h"

int main(void)
{
	Controller con("/dev/input/js0", NONBLOCK);
	Buttons btns;
	while(1) {
		if (con.readButtons(&btns) >= 0) {
			printf("Axes: ");
			for (int i = 0; i < 8; i++)
				printf("%2d:%6d ", i, btns.axis[i]);
			printf("Buttons: ");
			for (int i = 0; i < 11; i++)
				printf("%2d:%s ", i, btns.button[i] ? "on " : "off");
		}
		printf("\n");
		usleep(100000);
	}
	return 0;
}
		
