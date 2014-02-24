#include "vfh.h"

VFH::VFH()
{
	_densityThreshold = 1000;
	_spaceThreshold = 13;
	_VFHThreshold = 0;
	_constA = 2000;
	_constB = 1;
}

void VFH::setControlParameter(const int A, const int B)
{
	_constA = A;
	_constB = B;
}

void VFH::setAngleIndex(std::vector<double> angle)
{
	_angleVector = angle;
}

void VFH::setDensityThreshold(const int th)
{
	_densityThreshold = th;
}

int VFH::getDensityThreshold()
{
	return _densityThreshold;
}

void VFH::setSpaceThreshold(const int th)
{
	_spaceThreshold = th;
}

int VFH::getSpaceThreshold()
{
	return _spaceThreshold;
}

void VFH::setDetectionZone(const double angle, const int distance)
{
	_detectionRange = angle;
	_detectionDistance = distance;

	double upperLimit = _detectionRange / 2.0;
	double lowerLimit = -upperLimit;

	bool FLAGfound = false;
	for (unsigned int i=0; i<_angleVector.size(); i++) {
		if (_angleVector[i] <= upperLimit && _angleVector[i] >= lowerLimit) {
			if (!FLAGfound) {
				FLAGfound = true;
				_detectionRangeLowerIndex = i;
			}
		} else {
			if (FLAGfound) {
				_detectionRangeUpperIndex = i - 1;
				break;
			}
		}
	}
}

void VFH::getDetectionZone(double &angle, double &distance)
{
	angle = _detectionRange;
	distance = _detectionDistance;
}

void VFH::setVFHThreshold(const int th)
{
	_VFHThreshold = th;
}

int VFH::getVFHThreshold()
{
	return _VFHThreshold;
}

bool VFH::obstacleDetected()
{
	for (int i=_detectionRangeLowerIndex; i<=_detectionRangeUpperIndex; i++) {
		if (_distanceVector[i] < _detectionDistance) {
			return true;
		}
	}
	return false;
}

void VFH::update(std::vector<long> distance)
{
	sector.clear();
	_distanceVector = distance;

	_densityCounter = 0;
	_totalStep = 0;

	int sectorStartIndex = 0;
	int spaceCounter = 0;
	bool FLAGfound = false;

	struct Sector sectorTemp;
	unsigned int i;

	for (i=0; i<_distanceVector.size(); i++) {
		int VFHValue = _constA - (_constB * _distanceVector[i]);
		if (VFHValue < _VFHThreshold) {
			if (!FLAGfound) {
				FLAGfound = true;
				sectorStartIndex = i;
			}
			spaceCounter++;
		} else {
			if (FLAGfound) {
				if (spaceCounter > _spaceThreshold) {
					sectorTemp.width = spaceCounter;
					sectorTemp.angle = (_angleVector[sectorStartIndex] + _angleVector[i-1]) / 2.0; // (U + L) / 2
					sector.push_back(sectorTemp);
				}
				FLAGfound = false;
				spaceCounter = 0;
			}
		}
		if (_distanceVector[i] < _densityThreshold) {
			_densityCounter++;
		}
		_totalStep++;
	}

	/* 
	   In case the last sector extended all the way to the last index.
	*/
	if (FLAGfound) {
		if (spaceCounter > _spaceThreshold) {
			sectorTemp.width = spaceCounter;
			sectorTemp.angle = (_angleVector[sectorStartIndex] + _angleVector[i-1]) / 2.0; // (U + L) / 2 / 180
			sector.push_back(sectorTemp);
		}
	}
}

double VFH::getDensity()
{
	double den = (double)_densityCounter / (double)_totalStep;
	return den;
}

int VFH::getTotalStep()
{
	return _totalStep;
}

