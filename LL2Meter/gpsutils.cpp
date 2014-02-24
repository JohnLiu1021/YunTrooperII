#include "gpsutils.h"

double YT::GPS::Lat2Meter(double latRef, double latTarget)
{
	double refrence = ((latRef + latTarget)/2) * (M_PI/180.0);
	double sinValue = sin(refrence);
	double tmp1 = 1 - (e_square * sinValue * sinValue);
	tmp1 = sqrt(tmp1 * tmp1 * tmp1);
	tmp1 = 1 / tmp1;
	double r_curvature = a * (1-e_square) * tmp1;
	double m = (latTarget - latRef) * (M_PI/180.0) * r_curvature;
	return m;
}

double YT::GPS::Lon2Meter(double latRef, double lonRef, double lonTarget)
{
	double latRad = latRef * (M_PI / 180.0);
	double tmp1 = 1 - (e_square * sin(latRad) * sin(latRad));
	tmp1 = sqrt(tmp1);
	tmp1 = 1 / tmp1;
	
	double r_curvature = a * cos(latRad) * tmp1;
	double m = (lonTarget - lonRef) * (M_PI / 180.0) * r_curvature;
	return m;
}
double YT::GPS::calAngle(double latRef, double lonRef, double latTarget, double lonTarget)
{
	double m_x = Lat2Meter(latRef, latTarget);
	double m_y = Lon2Meter(latRef, lonRef, lonTarget);
	return atan2(m_y, m_x) * (180.0/M_PI);
}
