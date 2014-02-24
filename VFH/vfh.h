#include <vector>

class VFH {
public:
	VFH();

	struct Sector {
		int width;
		double angle;
	};
	std::vector<struct Sector> sector;

	void setControlParameter(const int, const int);
	void setAngleIndex(std::vector<double> angle);

	void setDensityThreshold(const int);
	int getDensityThreshold();

	void setSpaceThreshold(const int);
	int getSpaceThreshold();

	void setDetectionZone(const double, const int);
	void getDetectionZone(double &angle, double &distance);

	void setVFHThreshold(const int);
	int getVFHThreshold();

	bool obstacleDetected();
	void update(std::vector<long> distance);
	double getDensity();
	int getTotalStep();

private:
	int _densityCounter;
	int _totalStep;

	double _detectionRange;
	int _detectionDistance;
	int _detectionRangeUpperIndex;
	int _detectionRangeLowerIndex;

	int _VFHThreshold;
	int _densityThreshold;
	int _spaceThreshold;

	int _constA;
	int _constB;

	std::vector<long> _distanceVector;
	std::vector<double> _angleVector;
};
