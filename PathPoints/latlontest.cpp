#include "pathpoints.h"
//#define PATH "/home/john/Documents/BeagleBoneBlack/share/NavLog/20140220_019"
#define PATH "/tmp/20140220_019"

using namespace std;

int main(void)
{
	ifstream fin(PATH);
	if (!fin) {
		cout << "Error!" << endl;
		return -1;
	}

	int readCount = 0;

	while(1) {
		double buffer;
		double latM, lonM, latT, lonT;
		double yaw;
		if (fin >> buffer >> buffer >> latT >> lonT
			>> latM >> lonM >> buffer >> buffer >> yaw) {
			/*
			cout << setiosflags(ios::fixed) << setprecision(7) <<
				"Get!: " << latT << " " << lonT << " "<< latM << " "<< lonM << endl;
			*/
			readCount++;

			double m_x = PathPoints::Lat2Meter(latM, latT);
			double m_y = PathPoints::Lon2Meter(latM, lonM, lonT);
			double angle = -(atan2(m_y, m_x)) * (180/M_PI);
			double targetDirection = angle - yaw;

			cout << "m_x:" << m_x << ",\t"
			     << "m_y:" << m_y << ",\t"
			     << "angle:" << angle << ",\t"
			     << "yaw:" << yaw << ",\t"
			     << "targetDirection:" << targetDirection << endl;

		} else if (fin.eof()) {
			cout << "Reach EOF" << endl;
			break;
		} else {
			cout << "Error read" << endl;
			return -1;
		}
	}

	cout << "Read counter = " << readCount << endl;
	fin.close();
	return readCount;
}

		
