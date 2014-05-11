#include "objectrecognition_sv/point_pair.h"

unsigned int pointPairSI::maxHash;

const Eigen::Vector3f pointPairSI::SIPPF(const pcl::PointNormal & pointOne,const pcl::PointNormal & pointTwo)
{
	Eigen::Vector3f feature;

	// Get normalized distance vector
	Eigen::Vector3f distanceVectorNormalized;
	distanceVectorNormalized=(pointTwo.getVector3fMap()-pointOne.getVector3fMap()).normalized();

	// Compute angles (internal product)
	feature[0]=pointOne.getNormalVector3fMap().dot(distanceVectorNormalized);
	if(feature[0]<-1.000000) feature[0]=-1.000000;
	else if(feature[0]>=.999999) feature[0]=.999999;

	feature[1]=pointTwo.getNormalVector3fMap().dot(distanceVectorNormalized);
	if(feature[1]<-1.000000) feature[1]=-1.000000;
	else if(feature[1]>=0.999999) feature[1]=.999999;

	feature[2]=pointOne.getNormalVector3fMap().dot(pointTwo.getNormalVector3fMap());
	if(feature[2]<-1.000000) feature[2]=-1.000000;
	else if(feature[2]>=.999999) feature[2]=.999999;

	return feature;
}

const unsigned int pointPairSI::getHash(const pcl::PointNormal & secondaryPoint)
{
	Eigen::Vector3f feature=SIPPF(referencePoint,secondaryPoint);
	Eigen::Vector3d discreteFeature;

	// Discretize the output

	discreteFeature[0]=lutCosToAngle[ floor( (feature[1]+1.0) * static_cast<float>(cosResolution)*0.5) ];

	discreteFeature[1]=lutCosToAngle[ floor( (feature[2]+1.0) * static_cast<float>(cosResolution)*0.5) ];

	discreteFeature[2]=lutCosToAngle[ floor( (feature[3]+1.0) * static_cast<float>(cosResolution)*0.5) ];


	return (unsigned int) (discreteFeature[0]+(discreteFeature[1]*featureAngleBins)+(discreteFeature[2]*featureAngleBins*featureAngleBins));
}
