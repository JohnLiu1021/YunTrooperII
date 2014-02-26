#include <iostream>
#include "vfh2.h"

using namespace std;

int main(void)
{
	VFH vfh;
	vfh.open("/dev/ttyACM0");

	vfh.setScanningParameter(-90, 90, 4);
	vfh.setDangerDistance(100);
	vfh.setDetectionZone(RECT, 400, 1000);
	vfh.setDensityThreshold(600);
	vfh.setSpaceThreshold(450);

	if (!vfh.start()) {
		cout << "Error!" << endl;
	}

	for(int i=0; i<1; ) {
		vfh.update();
		cout << "Total step: " << vfh.getTotalStep() << endl;
		if (vfh.obstacleDetected()) {
			cout << "Detected obstalce!" << endl;
		}

		if (vfh.collisionDetected()) {
			cout << "Detected collision!" << endl;
		}
		cout << "Density: " << vfh.getDensity() << endl;

		if (vfh.sector.empty()) {
			cout << "There is no way to go...." << endl;
		} else {
			for (unsigned int i=0; i<vfh.sector.size(); i++) {
				cout << i << ": " << vfh.sector[i].width << ", " << vfh.sector[i].direction << endl;
			}
		}

	}
	return 0;
}
