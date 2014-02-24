#include <stdio.h>
#include "gpsutils.h"

int main(void)
{
	double latref = 23.4;
	double lonref = 120.3;

	double lattar = 23.5;
	double lontar = 120.2;

	double m_x = YT::GPS::Lat2Meter(latref, lattar);
	double m_y = YT::GPS::Lon2Meter(latref, lonref, lontar);
	double angle = YT::GPS::calAngle(latref, lonref, lattar, lontar);

	printf("m_x = %3.10f, m_y = %3.10f, angle = %3.10f\n", m_x, m_y, angle);
	return 0;
}
