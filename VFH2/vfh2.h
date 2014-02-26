#include <Urg_driver.h>

using namespace qrk;

enum DetectionZoneType {
	FAN,
	RECT
};

class VFH : public Urg_driver
{
public:
	VFH();
	~VFH();

	/* 
	   Following methods are configuration method.
	*/

	// Set the parameter of VFH algorithm.
	void setParameter(const double A, const double B);
	void getParameter(double &A, double &B);
	void getParameter(double *A, double *B);

	// Set the threshold distance of density counter.
	void setDensityThreshold(const int mm);
	int getDensityThreshold();

	// Set the available width of space in millimeter.
	void setSpaceThreshold(int mm);
	int getSpaceThreshold();

	// Set danger distance which is considered too close to the sensor, in millimeter.
	void setBodyWidth(const int mm);
	int getBodyWidth();

	// Set detection zone.
	void setDetectionZone(DetectionZoneType type, const double angle, const double meter);
	DetectionZoneType getDetectionZone(double &angle, double &meter);
	DetectionZoneType getDetectionZone(double *angle, double *meter);

	// Set scanning parameter of laser rangefinder
	void setScanningParameter(double minAngle, double maxAngle, int skipStep);
	void getScanningParameter(double &minAngle, double &maxAngle, int &skipStep);
	void getScanningParameter(double *minAngle, double *maxAngle, int *skipStep);

	/*
	   Utilization.
	*/
	// After configured parameters, calling this method
	bool start();

	// Calling this method to update the algorithm
	bool update();

	// If detected obstacle in the detection zone, return true, else false.
	bool obstacleDetected();

	// If it's about to collide with objects, return true, else false.
	bool collisionDetected();
	
	// Ask for the density value, which is between 0 and 1.
	double getDensity();

	// Get total step of the laser range finder.
	int getTotalStep();

	// Get the data read by laser rangefinder.
	void getMeasuredDistance(std::vector<long> &data, long *timestamp = NULL);

	// Sector vector, will only be recalculated by calling update() method.
	struct Sector {
		int width;
		double direction;
	};
	std::vector<struct Sector> sector;

private:
	double _paraA;
	double _paraB;

	struct DetectionZoneParameter {
		double para1;
		double para2;
		DetectionZoneType type;
	} _detectionPara;

	int _densityThreshold;
	int _spaceThreshold;
	int _bodyWidth;

	double _minAngle;
	double _maxAngle;
	int _skipStep;
	int _spaceStep;
	long _urgTimeStamp;

	std::vector<long> _measuredDistance;
	std::vector<double> _correspondAngle;
	std::vector<long> _detectionDistance;
};
