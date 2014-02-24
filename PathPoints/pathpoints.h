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
	size_t _currentIndex;

	static const double a;
	static const double f;
	static const double e_square;
	static const double b;

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

	int get(const int, double *, double *);
	int get(const int, double &, double &);

	static double Lat2Meter(double latRef, double latTarget);
	static double Lon2Meter(double latRef, double lonRef, double lonTarget);
	static double calculateAngle(double latRef, double lonRef, double LatTarget, double lonTarget);

};
#endif
