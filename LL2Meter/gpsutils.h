#ifndef GPS2METER_H
#define GPS2METER_H 1

#include <math.h>
#include <stdlib.h> // abs()

namespace YT {

class GPS {
private:
	const static double a = 6378137;
	const static double f = 1 / 298.257223563;
	const static double e_square = 0.0066943799901413164610;
	const static double b = 6356752.3142451792955399;

public:
	static double Lat2Meter(double latRef, double latTarget);
	static double Lon2Meter(double latRef, double lonRef, double lonTarget);
	static double calAngle(double latRef, double lonRef, double latTarget, double lonTarget);
};

};

#endif
