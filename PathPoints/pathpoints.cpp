#include "pathpoints.h"

PathPoints::PathPoints() : _currentIndex(-1)
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
	_currentIndex = -1;
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
}

int PathPoints::setCurrentIndex(const int index)
{
	if ((size_t)index >= _pathPoints.size() || index < -1)
		return -2;
	else
		_currentIndex = index;
	return _currentIndex;
}

int PathPoints::getCurrentIndex()
{
	return _currentIndex;
}

int PathPoints::getNext(double *lat, double *lon)
{
	if (_pathPoints.empty())
		return -2;

	_currentIndex++;
	std::vector<struct Point>::iterator it = _pathPoints.begin() + _currentIndex;
	if (it == _pathPoints.end()) {
		_currentIndex = -1;
	} else {
		*lat = (*it).latitude;
		*lon = (*it).longitude;
	}

	return (_currentIndex);
}

int PathPoints::getNext(double &lat, double &lon)
{	
	if (_pathPoints.empty())
		return -2;

	_currentIndex++;
	std::vector<struct Point>::iterator it = _pathPoints.begin() + _currentIndex;
	if (it == _pathPoints.end()) {
		_currentIndex = -1;
	} else {
		lat = (*it).latitude;
		lon = (*it).longitude;
	}

	return (_currentIndex);
}

int PathPoints::get(double *lat, double *lon, int index)
{
	if (index == -1)
		index = _currentIndex;

	if (_pathPoints.empty() || ((size_t)index) >= _pathPoints.size())
		return -1;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	*lat = (*it).latitude;
	*lon = (*it).longitude;
	return 0;
}

int PathPoints::get(double &lat, double &lon, int index)
{
	if (index == -1)
		index = _currentIndex;

	if (_pathPoints.empty() || (((size_t)index) >= _pathPoints.size()) || index < 0)
		return -2;

	std::vector<struct Point>::iterator it = _pathPoints.begin() + index;
	lat = (*it).latitude;
	lon = (*it).longitude;
	return 0;
}
