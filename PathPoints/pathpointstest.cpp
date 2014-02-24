#include "pathpoints.h"
#include <iostream>

using namespace std;

int main(void)
{
	PathPoints path;
	if (path.empty())
		cout << "Empty!" << endl;
	path.add(23.12312345104810934, 120.12312345123413848);
	path.add(23.12322345123412341, 120.12322435143412344);
	path.add(23.12332345, 120.12332435);
	path.add(23.12342345, 120.12342435);
	path.add(23.12352345, 120.12352345);
	path.add(23.12362345, 120.12362345);
	path.remove(1);

	int ret = path.writeToFile("/tmp/inputfile");
	if (ret > 0)
		cout << "Write " << ret << " points to file." << endl;

	int counter = 0;
	double lat;
	double lon;
	while(path.getNext(lat, lon) > 0) {
		double lat_curr = 23.123;
		double lon_curr = 120.123;
		cout << "Lat = " << lat << ", Lon = " << lon << endl;
		cout << "X = " << PathPoints::Lat2Meter(lat_curr, lat)
		     << " Y = " << PathPoints::Lon2Meter(lat_curr, lon_curr, lon) << endl;
		cout << "Angle diff = " << PathPoints::calculateAngle(lat_curr, lon_curr, lat, lon) << endl;
		cout << "Current index:" << path.getCurrentIndex() << endl;
		counter++;
	}
	cout << "counter = " << counter << ", size = " << path.size() << endl;

	cout << "Read from file /tmp/intpufile." << endl;
	
	ret = path.readFromFile("/tmp/inputfile");
	if (ret <= 0) {
		cout << "Error occured during reading data" << endl;
		return -1;
	}
	if (path.empty())
		cout << "Empty!" << endl;

	while(path.getNext(lat, lon) > 0) {
		double lat_curr = 23.123;
		double lon_curr = 120.123;
		cout.precision(11);
		cout << "Lat = " << lat << ", Lon = " << lon << endl;
		cout << "X = " << PathPoints::Lat2Meter(lat_curr, lat)
		     << " Y = " << PathPoints::Lon2Meter(lat_curr, lon_curr, lon) << endl;
		cout << "Angle diff = " << PathPoints::calculateAngle(lat_curr, lon_curr, lat, lon) << endl;
		cout << "Current index:" << path.getCurrentIndex() << endl;
		counter++;
	}



	return 0;
}
		


		
