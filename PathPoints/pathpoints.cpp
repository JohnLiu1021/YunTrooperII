#include "pathpoints.h"

const double PathPoints::a = 6378137;
const double PathPoints::f = 1 / 298.257223563;
const double PathPoints::e_square = 0.0066943799901413164610;
const double PathPoints::b = 6356752.3142451792955399;

PathPoints::PathPoints() : _currentIndex(0)
{
	_pathPoints.clear();
}

void PathPoints::add(const double lat, const double lon)
{
	struct Point temp;
	temp.latitude = lat;
	temp.longitude = lon;
	_pathPoints.push_back(temp);
}

int PathPoints::insert(const int index, const double lat, const double lon)
{
	if (_pathPoints.empty() && index != 0)
		return -1;

	struct Point temp;
	temp.latitude = lat;
	temp.longitude = lon;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	_pathPoints.insert(it, temp);
	return 0;
}

int PathPoints::remove(const int index)
{
	if ((size_t)index >= _pathPoints.size())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	_pathPoints.erase(it);
	return 0;
}

void PathPoints::clear()
{
	_pathPoints.clear();
	_currentIndex = 0;
}

int PathPoints::size()
{
	return _pathPoints.size();
}

bool PathPoints::empty()
{
	return _pathPoints.empty();
}

int PathPoints::readFromFile(const char *fileName)
{
	std::ifstream fin(fileName);
	if (!fin) {
		return -1;
	}

	// Clear all path points
	this->clear();
	
	int readCount = 0;
	while(1) {
		struct Point temp;
		if (fin >> temp.latitude >> temp.longitude) {
			_pathPoints.push_back(temp);
			readCount++;
		} else if (fin.eof()) {
			break;
		} else {
			return -1;
		}
	}
	fin.close();
	return readCount;

	/*
	FILE *stream = fopen(fileName, "r");
	if (!stream) {
		return -1;
	}

	// Clear all path if pathpoint exist
	if (!_pathPoints.empty())
		_pathPoints.clear();

	int readCounter = 0;
	while(1) {
		struct Point temp;
		int rd = fscanf(stream, "%lf %lf", &temp.latitude, &temp.longitude);
		if (rd == -1) {
			if(feof(stream)) {
				break;
			}
			else {
				return -1;
			}
		}
		_pathPoints.push_back(temp);
		readCounter++;
		printf("readCounter++\n");
	}
	fclose(stream);
	return readCounter;
	*/
}

int PathPoints::writeToFile(const char *fileName)
{
	if (_pathPoints.empty())
		return -1;
	
	std::ofstream fout(fileName);
	if (!fout) {
		return -1;
	}

	std::vector<struct Point>::iterator it;
	int writeCount = 0;
	for (it = _pathPoints.begin(); it != _pathPoints.end(); it++) {
		fout << std::setiosflags(std::ios::fixed) << std::setprecision(7)
		     << (*it).latitude << " "
		     << (*it).longitude << std::endl;

		writeCount++;
	}
	fout.close();
	return writeCount;

	/*
	FILE *stream = fopen(fileName, "w");
	if(!stream)
		return -1;

	std::vector<struct Point>::iterator it;
	int writeCounter = 0;
	for(it = _pathPoints.begin(); it != _pathPoints.end(); it++) {
		fprintf(stream, "%3.7f %3.7f\n", (*it).latitude, (*it).longitude);
		writeCounter++;
	}

	fclose(stream);
	return writeCounter;
	*/
}

int PathPoints::setCurrentIndex(const int index)
{
	_currentIndex = index;
	return _currentIndex;
}

int PathPoints::getCurrentIndex()
{
	return (_currentIndex-1);
}

int PathPoints::getNext(double *lat, double *lon)
{
	if (_pathPoints.empty())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + _currentIndex;
	if (it == _pathPoints.end()) {
		return 0;
	} else {
		*lat = (*it).latitude;
		*lon = (*it).longitude;
		_currentIndex++;
		return _currentIndex;
	}
}

int PathPoints::getNext(double &lat, double &lon)
{	
	if (_pathPoints.empty())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + _currentIndex;
	if (it == _pathPoints.end()) {
		return 0;
	} else {
		lat = (*it).latitude;
		lon = (*it).longitude;
		_currentIndex++;
		return _currentIndex;
	}
}

int PathPoints::get(const int index, double *lat, double *lon)
{
	if (_pathPoints.empty() || ((size_t)index) >= _pathPoints.size())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	*lat = (*it).latitude;
	*lon = (*it).longitude;
	return 0;
}

int PathPoints::get(const int index, double &lat, double &lon)
{
	if (_pathPoints.empty() || ((size_t)index) >= _pathPoints.size())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	lat = (*it).latitude;
	lon = (*it).longitude;
	return 0;
}

double PathPoints::Lat2Meter(double latRef, double latTarget)
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
double PathPoints::Lon2Meter(double latRef, double lonRef, double lonTarget)
{
	double latRad = latRef * (M_PI / 180.0);
	double tmp1 = 1 - (e_square * sin(latRad) * sin(latRad));
	tmp1 = sqrt(tmp1);
	tmp1 = 1 / tmp1;
	
	double r_curvature = a * cos(latRad) * tmp1;
	double m = (lonTarget - lonRef) * (M_PI / 180.0) * r_curvature;
	return m;
}
double PathPoints::calculateAngle(double latRef, double lonRef, double latTarget, double lonTarget)
{
	double m_x = PathPoints::Lat2Meter(latRef, latTarget);
	double m_y = PathPoints::Lon2Meter(latRef, lonRef, lonTarget);
	return atan2(m_y, m_x) * (180.0/M_PI);
}
