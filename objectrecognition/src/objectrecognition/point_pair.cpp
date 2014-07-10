#include "objectrecognition/point_pair.h"

unsigned int pointPair::angleBins;
float pointPair::angleStep;
float pointPair::angleStepInverted;
float pointPair::angleStepCos;
unsigned int pointPair::featureAngleBins;

//const unsigned int pointPair::cosResolution;
std::vector<unsigned int> pointPair::lutCosToAngle;
inline bool equalFloat(float a, float b, float epsilon)
{
    return fabs(a - b) < epsilon;
}

float pointPair::getRotation(const pcl::PointNormal & secondaryPoint,  const Eigen::Affine3f & transformToGlobal)
{
	Eigen::Vector3f _pointTwo=secondaryPoint.getVector3fMap();
	Eigen::Vector3f _pointTwoTransformed = transformToGlobal * _pointTwo;

	if(equalFloat(_pointTwoTransformed[1],0.0000000,0.000001)&&equalFloat(_pointTwoTransformed[2],0.0000000,0.000001))
	{
		return 0;
	}
	else
		return -std::atan2(_pointTwoTransformed[2],_pointTwoTransformed[1]);
}
