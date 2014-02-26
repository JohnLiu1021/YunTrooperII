#include "vfh2.h"
#include <cmath>
#include <stdio.h>

VFH::VFH()
{
	_paraA = 2000.0;
	_paraB = 1.0;

	_densityThreshold = 1000;
	_spaceThreshold = 400;
	_bodyWidth = 400;

	_detectionPara.para1 = 400;
	_detectionPara.para2 = 400;
	_detectionPara.type = RECT;

	_skipStep = 4;
	_minAngle = -120.0;
	_maxAngle = 120.0;

	_measuredDistance.clear();
	_correspondAngle.clear();
	_detectionDistance.clear();

	sector.clear();
}

VFH::~VFH(){};

void VFH::setParameter(const double A, const double B)
{
	_paraA = A;
	_paraB = B;
}

void VFH::getParameter(double &A, double &B)
{
	A = _paraA;
	B = _paraB;
}

void VFH::getParameter(double *A, double *B)
{
	*A = _paraA;
	*B = _paraB;
}

void VFH::setDensityThreshold(const int mm)
{
	_densityThreshold = mm;
}

int VFH::getDensityThreshold()
{
	return _densityThreshold;
}

void VFH::setSpaceThreshold(int mm)
{
	_spaceThreshold = mm;
}

int VFH::getSpaceThreshold()
{
	return _spaceThreshold;
}

void VFH::setDetectionZone(DetectionZoneType type, const double angle, const double mm)
{
	_detectionPara.type = type;
	_detectionPara.para1 = angle;
	_detectionPara.para2 = mm;
}

DetectionZoneType VFH::getDetectionZone(double &angle, double &mm)
{
	angle = _detectionPara.para1;
	mm = _detectionPara.para2;
	return _detectionPara.type;
}

DetectionZoneType VFH::getDetectionZone(double *angle, double *mm)
{
	*angle = _detectionPara.para1;
	*mm = _detectionPara.para2;
	return _detectionPara.type;
}

void VFH::setBodyWidth(const int mm)
{
	_bodyWidth = mm;
}

int VFH::getBodyWidth()
{
	return _bodyWidth;
}

void VFH::setScanningParameter(double minAngle, double maxAngle, int skipStep)
{
	_minAngle = minAngle;
	_maxAngle = maxAngle;
	_skipStep = skipStep;
}

void VFH::getScanningParameter(double &minAngle, double &maxAngle, int &skipStep)
{
	minAngle = _minAngle;
	maxAngle = _maxAngle;
	skipStep = _skipStep;
}

void VFH::getScanningParameter(double *minAngle, double *maxAngle, int *skipStep)
{
	*minAngle = _minAngle;
	*maxAngle = _maxAngle;
	*skipStep = _skipStep;
}

bool VFH::start()
{
	// Error checking.
	if (_minAngle > _maxAngle) {
		return false;
	} else if (_detectionPara.para1 <= 0 || _detectionPara.para2 <= 0) {
		return false;
	} else if (_paraA <= 0 || _paraB <= 0) {
		return false;
	}

	int minStep = deg2step(_minAngle);
	int maxStep = deg2step(_maxAngle);

	/* 
	   Transform each kind of detection zone into angle and distance,
	   also fill in the corresponding angle into the vector
	*/
	if (_detectionPara.type == FAN) {
		double upperAngleLimit = _detectionPara.para1 / 2;
		double lowerAngleLimit = -upperAngleLimit;

		for (int i=minStep; i<=maxStep; i+=_skipStep) {
			double angle = this->step2deg(i);
			_correspondAngle.push_back(angle);

			if (angle >= lowerAngleLimit && angle <= upperAngleLimit) {
				_detectionDistance.push_back((int)_detectionPara.para2);
			} else {
				_detectionDistance.push_back(0);
			}
		}
	} else if (_detectionPara.type == RECT) {
		for (int i=minStep; i<=maxStep; i+=_skipStep) {
			double angle = this->step2deg(i);
			_correspondAngle.push_back(angle);

			if (angle >= -89.0 && angle <= 89.0) {
				double angle_rad = angle * (M_PI/180.0);
				double front = _detectionPara.para2;
				double side = front * tan(angle_rad);
				if (fabs(side) > (_detectionPara.para1 / 2)) {
					side = _detectionPara.para1 / 2;
					front = side * tan((M_PI/2) - angle_rad);
				}
				
				double distance = sqrt((front*front) + (side*side));
				_detectionDistance.push_back((int)distance);
			} else {
				_detectionDistance.push_back(0);
			}
		}
	}

	/* 
	   Transform space threshold from mm to step according to paraA and paraB
	*/
	double zeroDist = _paraA / _paraB;
	double theta = 2 * (180.0 / M_PI) * asin((_spaceThreshold / 2) / zeroDist);
	_spaceStep = this->deg2step(theta);
	_spaceStep = lround((double)_spaceStep / (double)_skipStep);

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

bool VFH::update()
{
	if (!this->get_distance(_measuredDistance, &_urgTimeStamp)) {
		return false;
	}

	// Correct the range of data into range between 20 to 4000
	std::vector<long>::iterator it;
	for (it=_measuredDistance.begin(); it!=_measuredDistance.end(); it++) {
		if (*it > 4000 || *it < 20)
			*it = 4000;

		*it -= (_bodyWidth / 2);
	}

	// Calculate Vector Field Histogram.
	sector.clear();
	struct Sector SectorTemp;
	int freeSpaceCounter = 0;
	double sectorStartAngle = 0.0;
	bool Flag_foundFreeSector = false;

	std::vector<long>::iterator it_m = _measuredDistance.begin();
	std::vector<double>::iterator it_a = _correspondAngle.begin();

	for(; it_m!=_measuredDistance.end(); it_m++, it_a++) {

		double VFHValue = _paraA - (_paraB * (double)(*it_m));

		if (VFHValue <= 0) {
			if (!Flag_foundFreeSector) {
				Flag_foundFreeSector = true;
				sectorStartAngle = *it_a;
			}
			freeSpaceCounter++;
		} else {
			if (Flag_foundFreeSector) {
				if (freeSpaceCounter >= _spaceStep) {
					SectorTemp.width = freeSpaceCounter;
					SectorTemp.direction = (*(it_a - 1) + sectorStartAngle) / 2;
					sector.push_back(SectorTemp);
				}
				Flag_foundFreeSector = false;
				freeSpaceCounter = 0;
			}
		}
	}

	/*
	   In case the last sector extended all the way to the boundary.
	*/
	if (Flag_foundFreeSector) {
		if (freeSpaceCounter >= _spaceStep) {
			SectorTemp.width = freeSpaceCounter;
			SectorTemp.direction = (*(it_a - 1) + sectorStartAngle) / 2;
			sector.push_back(SectorTemp);
		}
	}

	return true;
}

bool VFH::obstacleDetected()
{
	std::vector<long>::iterator it_m;
	std::vector<long>::iterator it_d;

	int counter = 0;
	for (it_m = _measuredDistance.begin(),
	     it_d = _detectionDistance.begin();
	     it_m != _measuredDistance.end();
	     it_m++, it_d++) {
		counter++;
		if (*it_m < *it_d)
			return true;
	}
	return false;
}

bool VFH::collisionDetected()
{
	std::vector<long>::iterator it;
	for (it=_measuredDistance.begin(); it!=_measuredDistance.end(); it++) {
		if (*it < 0) {
			return true;
		}
	}
	return false;
}

double VFH::getDensity()
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

int VFH::getTotalStep()
{
	return (int)_measuredDistance.size();
}

void VFH::getMeasuredDistance(std::vector<long> &data, long *timestamp)
{
	data = _measuredDistance;
	if (timestamp)
		*timestamp = _urgTimeStamp;
}
