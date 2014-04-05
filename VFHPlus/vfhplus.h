#include <urg_cpp/Urg_driver.h>

using namespace qrk;

enum DetectionZoneType {
	FAN,
	RECT
};

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

	// Ask for the density value, which is between 0 and 1.
	double getDensity();

	// Get total step of the laser range finder.
	int getTotalStep();

	// Get the data read by laser rangefinder.
	void getMeasuredDistance(std::vector<long> &data, long *timestamp = NULL);

private:
	double _paraA, _paraB;
	double _lowerThreshold, _higherThreshold;

	double _u1, _u2, _u3;

	int _densityThreshold;
	int _bodyWidth;

	double _radius;

	double _minAngle;
	double _maxAngle;
	int _skipStep;
	double _angleStep;
	long _urgTimeStamp;

	double _phi_min;
	double _phi_max;

	std::vector<long> _measuredDistance;
	std::vector<double> _correspondAngle;
	std::vector<long> _maskedDistance;

	struct Sector {
		double start_rho;
		double end_rho;
		double start_angle;
		double end_angle;
		double width;
	};
	std::vector<struct Sector> _sector;

	double _lastDirection;

	inline double _findSectorWidth(struct Sector);
	inline double _unwrapRad(double rad);
	inline double _unwrapDeg(double deg);
};
