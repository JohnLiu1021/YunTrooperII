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

	_densityRange = M_PI / 4;
	_bodyWidth = 400;
	_angleThreshold = 30.0;

	_skipStep = 4;
	_minAngle = -120.0;
	_maxAngle = 120.0;

	_collisionDist = 450;

	_measuredDistance.clear();
	_correspondAngle.clear();
	_blockedDistance.clear();

	_sector.clear();
	_lastSteer = 0.0;
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

void VFHPlus::setDensityRange(const double deg)
{
	_densityRange = deg * (M_PI / 180.0);
}

double VFHPlus::getDensityRange()
{
	return _densityRange * (180.0 / M_PI);
}

void VFHPlus::setBodyWidth(const int mm)
{
	_bodyWidth = mm;
}

int VFHPlus::getBodyWidth()
{
	return _bodyWidth;
}

void VFHPlus::setAngleThreshold(const double deg)
{
	_angleThreshold = deg * (M_PI/180.0);
}

double VFHPlus::getAngleThreshold()
{
	return (_angleThreshold * (180.0 / M_PI));
}

void VFHPlus::setCollisionDistance(const double mm)
{
	_collisionDist = mm;
}

double VFHPlus::getCollisionDistance()
{
	return _collisionDist;
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

	//Calculate corresponding angle and masked distance.
	for (int i=minStep; i<=maxStep; i+=_skipStep) {
		double angle = this->step2rad(i);
		_correspondAngle.push_back(angle);

		// Blocked histogram with width
		angle = fabs(angle);
		double a = fabs(_radius * sin(angle));
		double b = (_radius*_radius) * (sin(angle) * sin(angle));
		b += ((_bodyWidth*_radius) + (_bodyWidth*_bodyWidth/4));

		double ans = a + sqrt(b);
		_blockedDistance.push_back(lround(ans));

		/* Previous density range */
		if ((angle >= -_densityRange) && (angle <= _densityRange)) {
			_densityPrevious.push_back(0);
		}
	}

	// Start the laser range finder.
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
	 
	// Correct the range of data into the range between 20 to 4000,
	std::vector<long>::iterator it = _measuredDistance.begin();
	for (; it!=_measuredDistance.end(); it++) {
		if (*it > 4000 || *it < 20)
			*it = 4000;
	}

	// Enter the loop of calculation for VFH
	_sector.clear();
	struct Sector SectorTemp;
	bool Flag_foundFreeSector = false;

	std::vector<long>::iterator it_measured = _measuredDistance.begin();
	std::vector<long>::iterator it_blocked = _blockedDistance.begin();
	std::vector<double>::iterator it_angle = _correspondAngle.begin();

	_phi_min = -M_PI;
	_phi_max = M_PI;
	double VFHPlusValue;
	double VFHPlusValue_p = _higherThreshold;

	// Closest distance and correspond angle
	double closestDist = 4000;
	double closestAngle = M_PI;;

	for(; it_measured!=_measuredDistance.end(); it_measured++, it_blocked++, it_angle++) {
		
		// Find closest distance
		if (*it_measured < closestDist) {
			closestDist = *it_measured;
			closestAngle = *it_angle;
		}

		// Finding Phi_max and Phi_min
		if ((*it_measured - *it_blocked) <= 0) {
			if (*it_angle < 0 && *it_angle > _phi_min) {
				_phi_min = *it_angle;
			} else if (*it_angle > 0 && *it_angle < _phi_max) {
				_phi_max = *it_angle;
			}
		}

		// Calculate VFHPlus value and find free sector
		VFHPlusValue = _paraA - (_paraB * (double)(*it_measured));

		// Hysteresis Threshold
		if ((VFHPlusValue > _lowerThreshold) && (VFHPlusValue < _higherThreshold))
			VFHPlusValue = VFHPlusValue_p; // Set with the previous value

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
				_sector.push_back(SectorTemp);
			}
		}
		VFHPlusValue_p = VFHPlusValue;
	}

	/*
	   In case the last sector extended all the way to the boundary.
	*/
	if (Flag_foundFreeSector) {
		SectorTemp.end_rho = *(it_measured - 1);
		SectorTemp.end_angle = *(it_angle - 1);
		Flag_foundFreeSector = false;
		_sector.push_back(SectorTemp);
	}

	/* Adding width */
	double halfWidth = (double)_bodyWidth / 2;
	if (closestDist < halfWidth)
		closestDist = halfWidth;

	double closestUpperLimit = closestAngle + asin(halfWidth / closestDist);
	double closestLowerLimit = closestAngle - asin(halfWidth / closestDist);

	std::vector<struct Sector>::iterator it_temp;
	for (it_temp=_sector.begin(); it_temp!=_sector.end(); it_temp++) {
		
		double value;

		value = halfWidth / it_temp->start_rho;
		if (value <= 1 && value >= -1) {
			value = it_temp->start_angle + asin(value);
			if (value >= closestLowerLimit && value <= closestUpperLimit)
				it_temp->start_angle = closestUpperLimit;
			else
				it_temp->start_angle = value;

		} else {
			it_temp->start_angle = M_PI;
		}

		value = halfWidth / it_temp->end_rho;
		if (value <= 1 && value >= -1) {
			value = it_temp->end_angle - asin(value);
			if (value >= closestLowerLimit && value <= closestUpperLimit)
				it_temp->end_angle = closestLowerLimit;
			else
				it_temp->end_angle = value;

		} else {
			it_temp->end_angle = -M_PI;
		}
	}


/* Debug
	printf("Phi max: %3.2f, Phi min: %3.2f\n", _phi_max*(180/M_PI), _phi_min*(180/M_PI));
	int i = 0;
	for (it_temp=_sector.begin(); it_temp!=_sector.end(); it_temp++) {
		i++;
		printf("#%d - Start rho: %3.1f, End rho: %3.1f, Start angle: %3.1f, End angle: %3.1f\n",
		       i, it_temp->start_rho, it_temp->end_rho, 
		       (it_temp->start_angle)*(180/M_PI), (it_temp->end_angle)*(180/M_PI));

	}
*/
	
	return true;
}


double VFHPlus::calculateDirection(double targetDirection)
{
	double targetRad = targetDirection * (M_PI / 180);

	std::vector<double> candidateAngle;
	candidateAngle.clear();

	// Find candidate angle
	std::vector<struct Sector>::iterator it;
	for(it=_sector.begin(); it!=_sector.end(); it++) {
		if (it->start_angle <= it->end_angle) {
			if ((it->end_angle - it->start_angle) <= _angleThreshold) {
				candidateAngle.push_back((it->start_angle + it->end_angle) / 2.0);
			} else {
				candidateAngle.push_back(it->start_angle);
				candidateAngle.push_back(it->end_angle);
				if (targetRad > it->start_angle && targetRad < it->end_angle)
					candidateAngle.push_back(targetRad);
			}
		}
	}

	// Calculate cost function
	std::vector<double>::iterator it_a;
	double minValue = M_PI;
	double preferDirection = 0;

	for (it_a=candidateAngle.begin(); it_a!=candidateAngle.end(); it_a++) {
		if (*it_a > _phi_min && *it_a < _phi_max) {
			double targetDiff = fabs(_unwrapRad(targetRad - *it_a));
			double currentDiff = fabs(*it_a);
			double lastDiff = fabs(_lastSteer - *it_a);
			double value = (_u1 * targetDiff) + (_u2 * currentDiff) + (_u3 * lastDiff);
			if (value < minValue) {
				minValue = value;
				preferDirection = *it_a;
			}
		}
	}
	
	if (minValue == M_PI) {
		return 10000;
	} else {
		_lastSteer = preferDirection;
		return (preferDirection * (180 / M_PI));
	}
}

bool VFHPlus::collisionDetected(double direction)
{
	// No candidate angle
	if (direction > 180)
		direction = 0;

	if (direction > 45.0)
		direction = 45.0;
	else if (direction < -45.0)
		direction = -45.0;
	direction *= (M_PI / 180.0);
	
	double bodyRadius = _bodyWidth / 2;

	double a = atan2(bodyRadius, _collisionDist);
	double angleLower = direction - a;
	double angleHigher = direction + a;

	std::vector<long>::iterator it_d;
	std::vector<double>::iterator it_a;
	for (it_d=_measuredDistance.begin(),it_a=_correspondAngle.begin();
	     it_d != _measuredDistance.end();
	     it_d++,it_a++) {
		// First stage
		if ((*it_a >= -(65.0*(M_PI/180))) && (*it_a <= (65.0*(M_PI/180)))) {
			if (*it_d < bodyRadius)
				return true;
		}

		// Second stage
		if ((*it_a >= angleLower) && (*it_a <= angleHigher)) {
			if (*it_d < _collisionDist)
				return true;
		}
	}

	return false;
}

double VFHPlus::getDensity()
{
	int stepCounter = 0;
	int diffStepCounter = 0;
	int densityCounter = 0;
	int densityDiffCounter = 0;

	std::vector<long>::iterator it_d;
	std::vector<double>::iterator it_a;
	std::vector<long>::iterator it_p;

	for (it_d=_measuredDistance.begin(),it_a=_correspondAngle.begin(),it_p=_densityPrevious.begin();
	     it_d!=_measuredDistance.end();
	     it_d++, it_a++) {
		stepCounter++;
		densityCounter += *it_d;
		if ((*it_a >= -_densityRange) && (*it_a <= _densityRange)) {
			diffStepCounter++;
			int densityDiff = *it_d - *it_p;
			if (densityDiff < 0) {
				densityDiffCounter += densityDiff;
			}
			*it_p = *it_d;
			it_p++;
		}
	}

	double density = 1.0 - ((double)densityCounter / (double)(4000 * stepCounter));
	density += -((double)densityDiffCounter / (double)(200 * diffStepCounter));

	if (density > 1.0)
		return 1.0;
	else
		return density;

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

void VFHPlus::getCorrespondAngle(std::vector<double> &data)
{
	data = _correspondAngle;
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
