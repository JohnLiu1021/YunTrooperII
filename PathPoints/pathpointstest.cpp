#include "pathpoints.h"
#include <iostream>

using namespace std;

int main(void)
{
	PathPoints path;
	if (path.empty())
		cout << "Empty!" << endl;
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
	for(int i=0; i<20; i++) {
		path.getNext(lat, lon);
		cout << "Current Index: " << path.getCurrentIndex();
		cout << " Lat = " << lat << " Lon = " << lon << endl;
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

	path.setCurrentIndex(-1);
	while(path.getNext(lat, lon) >= 0) {
		cout.precision(11);
		cout << "Lat = " << lat << ", Lon = " << lon << endl;
		counter++;
	}



	return 0;
}
		


		
