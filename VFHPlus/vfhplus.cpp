#include "vfhplus.h"
#include <cmath>
#include <stdio.h>
//#define DEBUG 1

VFHPlus::VFHPlus()
{
	_paraA = 2000.0;
	_paraB = 1.0;

	_lowerThreshold = 0;
	_higherThreshold = 200;

	_phi_min = -M_PI;
	_phi_max = M_PI;

	_densityThreshold = 1000;
	_bodyWidth = 400;

	_skipStep = 4;
	_minAngle = -120.0;
	_maxAngle = 120.0;

	_measuredDistance.clear();
	_correspondAngle.clear();
	_maskedDistance.clear();

	_sector.clear();
	_lastDirection = 0.0;
}

VFHPlus::~VFHPlus(){};

void VFHPlus::setVFHPlusParameter(const double A, const double B)
{
	_paraA = A;
	_paraB = B;
}

void VFHPlus::getVFHPlusParameter(double &A, double &B)
{
	A = _paraA;
	B = _paraB;
}

void VFHPlus::getVFHPlusParameter(double *A, double *B)
{
	*A = _paraA;
	*B = _paraB;
}

void VFHPlus::setVFHThreshold(double low, double high)
{
	_lowerThreshold = low;
	_higherThreshold = high;
}

void VFHPlus::getVFHThreshold(double &low, double &high)
{
	low = _lowerThreshold;
	high = _higherThreshold;
}

void VFHPlus::getVFHThreshold(double *low, double *high)
{
	*low = _lowerThreshold;
	*high = _higherThreshold;
}

void VFHPlus::setCostFuncParameter(const double u1, const double u2, const double u3)
{
	_u1 = u1;
	_u2 = u2;
	_u3 = u3;
}

void VFHPlus::getCostFuncParameter(double &u1, double &u2, double &u3)
{
	u1 = _u1;
	u2 = _u2;
	u3 = _u3;
}

void VFHPlus::getCostFuncParameter(double *u1, double *u2, double *u3)
{
	*u1 = _u1;
	*u2 = _u2;
	*u3 = _u3;
}

void VFHPlus::setDensityThreshold(const int mm)
{
	_densityThreshold = mm;
}

int VFHPlus::getDensityThreshold()
{
	return _densityThreshold;
}

void VFHPlus::setBodyWidth(const int mm)
{
	_bodyWidth = mm;
}

int VFHPlus::getBodyWidth()
{
	return _bodyWidth;
}

void VFHPlus::setRadiusOfCurvature(double radius)
{
	_radius = radius;
}

double VFHPlus::getRadiusOfCurvature()
{
	return _radius;
}

void VFHPlus::setScanningParameter(double minAngle, double maxAngle, int skipStep)
{
	_minAngle = minAngle;
	_maxAngle = maxAngle;
	_skipStep = skipStep;
}

void VFHPlus::getScanningParameter(double &minAngle, double &maxAngle, int &skipStep)
{
	minAngle = _minAngle;
	maxAngle = _maxAngle;
	skipStep = _skipStep;
}

void VFHPlus::getScanningParameter(double *minAngle, double *maxAngle, int *skipStep)
{
	*minAngle = _minAngle;
	*maxAngle = _maxAngle;
	*skipStep = _skipStep;
}

bool VFHPlus::start()
{
	// Error checking.
	if (_minAngle > _maxAngle) {
		return false;
	} else if (_paraA <= 0 || _paraB <= 0) {
		return false;
	}

	int minStep = deg2step(_minAngle);
	int maxStep = deg2step(_maxAngle);
	_angleStep = step2rad(_skipStep) - step2rad(0);

	/*
	   Calculate corresponding angle and masked distance.
	*/
	for (int i=minStep; i<=maxStep; i+=_skipStep) {
		double angle = this->step2rad(i);
		_correspondAngle.push_back(angle);

		/* 
		   Masked histogram without width
		double rho = fabs(2*_radius*sin(angle));
		_maskedDistance.push_back(lround(rho));
		*/

		/* 
		   Masked histogram with width
		*/
		angle = fabs(angle);
		double a = fabs(_radius * sin(angle));
		double b = (_radius*_radius) * (sin(angle) * sin(angle));
		b += ((_bodyWidth*_radius) + (_bodyWidth*_bodyWidth/4));

		double ans = a + sqrt(b);
		_maskedDistance.push_back(lround(ans));
	}

	/* 
	   Start the laser range finder.
	*/
	if (!this->set_scanning_parameter(minStep, maxStep, _skipStep)) {
		return false;
	}
	if (!this->start_measurement(Urg_driver::Distance)) {
		return false;
	}

	return true;
}

bool VFHPlus::update()
{
	if (!this->get_distance(_measuredDistance, &_urgTimeStamp)) {
		return false;
	}

	/* 
	   Correct the range of data into the range between 20 to 4000,
	*/
	std::vector<long>::iterator it = _measuredDistance.begin();
	for (; it!=_measuredDistance.end(); it++) {
		if (*it > 4000 || *it < 20)
			*it = 4000;
	}

	// Calculate Vector Field Histogram.
	_sector.clear();
	struct Sector SectorTemp;
	bool Flag_foundFreeSector = false;

	std::vector<long>::iterator it_measured = _measuredDistance.begin();
	std::vector<long>::iterator it_masked = _maskedDistance.begin();
	std::vector<double>::iterator it_angle = _correspondAngle.begin();

	_phi_min = -M_PI;
	_phi_max = M_PI;
	double VFHPlusValue;
	double VFHPlusValue_p = _higherThreshold;

	for(; it_measured!=_measuredDistance.end(); it_measured++, it_masked++, it_angle++) {
		/*
		   Finding Phi_max and Phi_min
		*/
		if ((*it_measured - *it_masked) <= 0) {
			if (*it_angle < 0 && *it_angle > _phi_min) {
				_phi_min = *it_angle;
			} else if (*it_angle > 0 && *it_angle < _phi_max) {
				_phi_max = *it_angle;
			}
		}

		/* 
		   Calculate VFHPlus value and find free sector
		*/
		VFHPlusValue = _paraA - (_paraB * (double)(*it_measured));

		if ((VFHPlusValue > _lowerThreshold) && (VFHPlusValue < _higherThreshold))
			VFHPlusValue = VFHPlusValue_p;

		if (VFHPlusValue <= _lowerThreshold) {
			if (!Flag_foundFreeSector) {
				Flag_foundFreeSector = true;
				if (it_measured != _measuredDistance.begin()) {
					SectorTemp.start_rho = *(it_measured - 1);
					SectorTemp.start_angle = *(it_angle - 1);
				} else {
					SectorTemp.start_rho = *it_measured;
					SectorTemp.start_angle = *it_angle;
				}
			}
		} else if (VFHPlusValue >= _higherThreshold) {
			if (Flag_foundFreeSector) {
				SectorTemp.end_rho = *(it_measured);
				SectorTemp.end_angle = *(it_angle);
				Flag_foundFreeSector = false;
				if ((SectorTemp.width=_findSectorWidth(SectorTemp)) >= _bodyWidth)
					_sector.push_back(SectorTemp);
			}
		}
		VFHPlusValue_p = VFHPlusValue;
	}

	/*
	   In case the last sector extended all the way to the boundary.
	*/
	if (Flag_foundFreeSector) {
		SectorTemp.end_rho = *(it_measured-1);
		SectorTemp.end_angle = *(it_angle-1);
		Flag_foundFreeSector = false;
		if ((SectorTemp.width=_findSectorWidth(SectorTemp)) >= _bodyWidth)
			_sector.push_back(SectorTemp);
	}

	/* Adding width */
	std::vector<struct Sector>::iterator it_temp;
	for (it_temp=_sector.begin(); it_temp!=_sector.end(); it_temp++) {
		double halfWidth = (double)_bodyWidth / 2;
		it_temp->start_angle += asin(halfWidth / it_temp->start_rho);
		it_temp->end_angle -= asin(halfWidth / it_temp->end_rho);
	}


#ifdef DEBUG	
	printf("Phi max: %3.2f, Phi min: %3.2f\n", _phi_max*(180/M_PI), _phi_min*(180/M_PI));
	int i = 0;
	for (it_temp=_sector.begin(); it_temp!=_sector.end(); it_temp++) {
		i++;
		printf("#%d - Start rho: %3.1f, End rho: %3.1f, Start angle: %3.1f, End angle: %3.1f, width: %4f\n",
		       i, it_temp->start_rho, it_temp->end_rho, 
		       (it_temp->start_angle)*(180/M_PI), (it_temp->end_angle)*(180/M_PI),
		       it_temp->width);

	}
#endif

	return true;
}


double VFHPlus::calculateDirection(double targetDirection)
{
	double targetRad = targetDirection * (M_PI / 180);

	std::vector<double> candidateAngle;
	candidateAngle.clear();

	std::vector<struct Sector>::iterator it;

	/* Find candidate angle
	for(it=_sector.begin(); it!=_sector.end(); it++) {
		if (it->start_angle > it->end_angle) {
			continue;
		} else  {
			for(double angleIndex=it->start_angle; angleIndex<=it->end_angle; angleIndex+=_angleStep) {
				candidateAngle.push_back(angleIndex);
			}
		}
	}
	*/
	/* Paper's way */
	for(it=_sector.begin(); it!=_sector.end(); it++) {
		if (it->start_angle > it->end_angle) {
			continue;
		} else  {
			candidateAngle.push_back(it->start_angle);
			candidateAngle.push_back(it->end_angle);
			if (targetRad > it->start_angle && targetRad < it->end_angle)
				candidateAngle.push_back(targetRad);
		}
	}
	

	// Calculate cost function
	std::vector<double>::iterator it_a;
	double minValue = 1.0;
	double preferDirection = 0;

	for (it_a=candidateAngle.begin(); it_a!=candidateAngle.end(); it_a++) {
		if (*it_a > _phi_min && *it_a < _phi_max) {
			double targetDiff = fabs(_unwrapRad(targetRad - *it_a)) / M_PI;
			double currentDiff = fabs(*it_a) / M_PI;
			double lastDiff = fabs(_lastDirection - *it_a) / M_PI;
			double value = (_u1 * targetDiff) + (_u2 * currentDiff) + (_u3 * lastDiff);
			if (value < minValue) {
				minValue = value;
				preferDirection = *it_a;
			}
		}
	}
	
	if (minValue == 1.0) {
		return 10000;
	} else {
		_lastDirection = preferDirection;
		return (preferDirection * (180 / M_PI));
	}
}

double VFHPlus::getDensity()
{
	int totalStep = (int)_measuredDistance.size();
	int stepCounter = 0;

	std::vector<long>::iterator it;
	for (it=_measuredDistance.begin(); it!=_measuredDistance.end(); it++) {
		if (*it < _densityThreshold)
			stepCounter++;
	}

	double density = (double)stepCounter / (double)totalStep;
	return density;
}

int VFHPlus::getTotalStep()
{
	return (int)_measuredDistance.size();
}

void VFHPlus::getMeasuredDistance(std::vector<long> &data, long *timestamp)
{
	data = _measuredDistance;
	if (timestamp)
		*timestamp = _urgTimeStamp;
}

inline double VFHPlus::_findSectorWidth(struct Sector s)
{
	double A = s.start_rho;
	double B = s.end_rho;
	double angle_diff = fabs(s.start_angle - s.end_angle);
	double C_square = A*A + B*B - 2*A*B*cos(angle_diff);

	return sqrt(C_square);
}

inline double VFHPlus::_unwrapRad(double rad)
{
	if (rad > M_PI) {
		rad -= (2*M_PI);
	} else if (rad < -M_PI) {
		rad += (2*M_PI);
	}
	return rad;
}
	
inline double VFHPlus::_unwrapDeg(double deg)
{
	if (deg > 180) {
		deg -= 360;
	} else if (deg < -180) {
		deg += 360;
	}
	
	return deg;
}

