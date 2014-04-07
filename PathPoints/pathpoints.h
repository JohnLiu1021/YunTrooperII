#ifndef PATHPOINTS_H
#define PATHPOINTS_H 1

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

class PathPoints {
private:
	struct Point {
		double latitude;
		double longitude;
	};

	std::vector<struct Point> _pathPoints;
	int _currentIndex;
public:
	PathPoints();

	void add(const double, const double);
	int insert(const int, const double, const double);
	int remove(const int);
	void clear();
	int size();
	bool empty();

	int readFromFile(const char *);
	int writeToFile(const char *);

	int setCurrentIndex(const int);
	int getCurrentIndex();
	
	int getNext(double *, double *);
	int getNext(double &, double &);

	int get(double *, double *, int index = -1);
	int get(double &, double &, int index = -1);

};
#endif
