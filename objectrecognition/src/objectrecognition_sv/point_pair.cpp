#include "objectrecognition_sv/point_pair.h"
#include <iostream>
#include <fstream>

unsigned int pointPairSV::distanceBins;
unsigned int pointPairSV::maxHash;

/*std::ostream& operator<<(std::ostream& os, const pointPairSV& PPF)
{
    os << PPF.ppf.transpose() << " " << PPF.discrete_ppf.transpose();
    return os;
}*/

const Eigen::Vector4f pointPairSV::PPF(const pcl::PointNormal & pointOne,const pcl::PointNormal & pointTwo)
{
	Eigen::Vector4f feature;

	Eigen::Vector3f normal_1=pointOne.getNormalVector3fMap();//
	normal_1.normalize();

	Eigen::Vector3f normal_2=pointTwo.getNormalVector3fMap();//
	normal_2.normalize();
	/// Compute euclidean distance between points
	Eigen::Vector3f distanceVector=pointTwo.getVector3fMap()-pointOne.getVector3fMap();
	feature[0]=(distanceVector).norm();
	// Get temporary normalized distance vector
	Eigen::Vector3f distanceVectorNormalized;
	distanceVectorNormalized=distanceVector.normalized();

	// Compute angles (internal product)
	feature[1]=normal_1.dot(distanceVectorNormalized);
	if(feature[1]<-.99999) feature[1]=-1.0;
	else if(feature[1]>=.99999) feature[1]=1.0;

	feature[2]=normal_2.dot(distanceVectorNormalized);
	if(feature[2]<-.99999) feature[2]=-1.0;
	else if(feature[2]>=0.99999) feature[2]=1.0;

	feature[3]=normal_1.dot(normal_2);
	if(feature[3]<-.99999) feature[3]=-1.0;
	else if(feature[3]>=.99999) feature[3]=1.0;
	//std::cout << normal_1 << std::endl;
	return feature;
}

/*const Eigen::Vector4f pointPairSV::PPF(const pcl::PointNormal & pointOne,const pcl::PointNormal & pointTwo)
{
	Eigen::Vector4f feature;

	/// Compute euclidean distance between points
	Eigen::Vector3f distanceVector=pointTwo.getVector3fMap()-pointOne.getVector3fMap();
	feature[0]=distanceVector.norm();

	// Get temporary normalized distance vector
	Eigen::Vector3f distanceVectorNormalized;
	distanceVectorNormalized=distanceVector/dist;

	// Compute angles (internal product)
	feature[1]=pointOne.getNormalVector3fMap().dot(distanceVectorNormalized);
	//if(feature[1]<-.999) feature[1]=-.999;
	//else if(feature[1]>=.999) feature[1]=.999;

	feature[2]=pointTwo.getNormalVector3fMap().dot(distanceVectorNormalized);
	//if(feature[2]<-.999) feature[2]=-.999;
	//else if(feature[2]>=0.999) feature[2]=.999;

	feature[3]=pointOne.getNormalVector3fMap().dot(pointTwo.getNormalVector3fMap());
	//if(feature[3]<-.999) feature[3]=-.999;
	//else if(feature[3]>=.999) feature[3]=.999;

	return feature;
}
*/


const unsigned int pointPairSV::getHash(const pcl::PointNormal & secondaryPoint, const float & distanceStepInverted)
{
	Eigen::Vector4f feature=PPF(referencePoint,secondaryPoint);
	//ppf=feature;

	Eigen::Vector4d discreteFeature;

	// Discretize the output
	discreteFeature[0]=round(feature[0]*distanceStepInverted);
	if(discreteFeature[0]==distanceBins) --discreteFeature[0];

	discreteFeature[1]=lutCosToAngle[ round( (feature[1]+1.0) * static_cast<float>(cosResolution)*0.5) ];
	if(discreteFeature[1]==featureAngleBins) --discreteFeature[1];

	discreteFeature[2]=lutCosToAngle[ round( (feature[2]+1.0) * static_cast<float>(cosResolution)*0.5) ];
	if(discreteFeature[2]==featureAngleBins) --discreteFeature[2];

	discreteFeature[3]=lutCosToAngle[ round( (feature[3]+1.0) * static_cast<float>(cosResolution)*0.5) ];
	if(discreteFeature[3]==featureAngleBins) --discreteFeature[3];

	 // std::ofstream myfile;
//	  myfile.open ("example.txt",std::ios::app);
//	  myfile  << feature[0] <<" " <<discreteFeature[0]<< std::endl;
//	  myfile  <<acos(feature[3])*RAD_TO_DEG <<" " <<discreteFeature[3]<< std::endl;

//	  myfile.close();
	  //discrete_ppf=discreteFeature;
	//std::cout << "discreteFeature[0]: "<< discreteFeature[0] << " discreteFeature[1]:" << discreteFeature[1] << " discreteFeature[2]:" << discreteFeature[2] << " discreteFeature[3]: " << discreteFeature[3] << " " << feature[3] << std::endl;
	return (unsigned int) (discreteFeature[1]+(discreteFeature[2]*featureAngleBins)+(discreteFeature[3]*featureAngleBins*featureAngleBins)+(discreteFeature[0]*featureAngleBins*featureAngleBins*featureAngleBins));
}
