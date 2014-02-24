#include "vfh.h"
#include <Urg_driver.h>
#include <stdio.h>

using namespace qrk;

int main(void)
{
	VFH vfh;
	Urg_driver urg;
	if (!urg.open("/dev/ttyACM0")) {
		fprintf(stderr, "Error open : urg device\n");
		return -1;
	}
	urg.start_measurement(Urg_driver::Distance);
	int min_step = urg.min_step();
	int max_step = urg.max_step();

	std::vector<double> angle;
	angle.clear();

	for (int i=min_step; i<=max_step; i++) {
		angle.push_back(urg.step2deg(i));
	}

	int count = 0;
	for (int j=min_step; j<=max_step; j+=4, count++) {
		printf("j = %d\n", j);
	}
	printf("count = %d\n", count);

	vfh.setAngleIndex(angle);
	vfh.setDetectionZone(60.0, 2000);

	while(1) {
		std::vector<long> data;
		long time_stamp = 0;
		if (!urg.get_distance(data, &time_stamp)) {
			perror("read : urg device");
			return -1;
		}
		std::vector<long>::iterator it;
		for (it = data.begin(); it != data.end(); it++) {
			if (*it > 4000 || *it < 20)
				*it = 4000;
		}

		vfh.update(data);
		if (vfh.obstacleDetected())
			printf("Obstalce detected!\n");
		for (int i=0; i<vfh.sector.size(); i++) {
			printf("Sector %d : width = %d, angle = %3.4f\n", i, vfh.sector[i].width, vfh.sector[i].angle);
		}
		printf("density = %f\n", vfh.getDensity());
	}

	return 0;
}
