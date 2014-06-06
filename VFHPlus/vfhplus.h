#include <urg_cpp/Urg_driver.h>

using namespace qrk;

class VFHPlus : public Urg_driver
{
public:
	VFHPlus();
	~VFHPlus();

	/* 
	   Following methods are configuration method.
	*/

	// Set the parameter of VFHPlus algorithm.
	void setVFHPlusParameter(const double A, const double B);
	void getVFHPlusParameter(double &A, double &B);
	void getVFHPlusParameter(double *A, double *B);

	// Set VFH higher and lower threshold
	void setVFHThreshold(double low, double high);
	void getVFHThreshold(double &low, double &high);
	void getVFHThreshold(double *low, double *high);

	// Set the parameter of cost function
	void setCostFuncParameter(const double u1, const double u2, const double u3);
	void getCostFuncParameter(double &u1, double &u2, double &u3);
	void getCostFuncParameter(double *u1, double *u2, double *u3);

	// Set the threshold distance of density counter.
	void setDensityThreshold(const int mm);
	int getDensityThreshold();

	// Set danger distance which is considered too close to the sensor, in millimeter.
	void setBodyWidth(const int mm);
	int getBodyWidth();

	// Set the threshold between wide and narrow sector.
	void setAngleThreshold(const double deg);
	double getAngleThreshold();

	// Set the fan collision detecting area with spanning angle and distance
	void setCollisionArea(const double deg, const double distance);
	void setCollisionArea(std::vector<long> v);
	void getCollisionArea(double *deg, double *distance);
	void getCollisionArea(double &deg, double &distance);

	// Set radius of curvature
	void setRadiusOfCurvature(double radius);
	double getRadiusOfCurvature();

	// Set scanning parameter of laser rangefinder
	void setScanningParameter(double minAngle, double maxAngle, int skipStep);
	void getScanningParameter(double &minAngle, double &maxAngle, int &skipStep);
	void getScanningParameter(double *minAngle, double *maxAngle, int *skipStep);

	/*
	   Utilization.
	*/
	// After configured parameters, calling this method
	bool start();

	// Calling this method to update the measurement
	bool update();

	// Calling this method to calculate the optimized direction
	double calculateDirection(double targetDirection);

	// Detect possible collision
	bool collisionDetected();

	// Ask for the density value, which is between 0 and 1.
	double getDensity();

	// Get total step of the laser range finder.
	int getTotalStep();

	// Get the data read by laser rangefinder.
	void getMeasuredDistance(std::vector<long> &data, long *timestamp = NULL);

	// Get the correspond angle.
	void getCorrespondAngle(std::vector<double> &data);

private:
	// Parameter of VFH formula: V = A - B*D
	double _paraA, _paraB;

	// Paremeter of hysteresis filter 
	double _lowerThreshold, _higherThreshold;

	// Paremeter of weighting function
	double _u1, _u2, _u3;

	// Threshold of environment density calculation
	int _densityThreshold;

	// Body width
	int _bodyWidth;

	// Car's radius of curvature
	double _radius;

	// Threshold of open angle of sector
	double _angleThreshold; 

	// Angle and distance parameter of collision detecting area
	double _spanningAngle;
	double _extendedDistance;

	// Scanning Parameter of laser range finder
	double _minAngle;
	double _maxAngle;
	int _skipStep;
	double _angleStep;
	
	// Time stamp of laser range finder	
	long _urgTimeStamp;

	// Limit of steering angle
	double _phi_min;
	double _phi_max;

	// Storing last steering command
	double _lastSteer;

	// Free sector structure
	struct Sector {
		double start_rho;
		double end_rho;
		double start_angle;
		double end_angle;
		double width;
	};
	std::vector<struct Sector> _sector;

	std::vector<long> _measuredDistance;
	std::vector<double> _correspondAngle;
	std::vector<long> _maskedDistance;
	std::vector<long> _collisionDistance;

	inline double _findSectorWidth(struct Sector);
	inline double _unwrapRad(double rad);
	inline double _unwrapDeg(double deg);

};
