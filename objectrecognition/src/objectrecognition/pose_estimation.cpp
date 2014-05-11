#include "objectrecognition/pose_estimation.h"

// static variables
float poseEstimation::referencePointsPercentage;
float poseEstimation::accumulatorPeakThreshold;
bool poseEstimation::filterOn;

poseEstimation::poseEstimation()
{	
	cloudNormals=pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>);
	cloudWithNormalsDownSampled=pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>);
	referencePointsIndices = pcl::PointIndices::Ptr (new pcl::PointIndices);
}

poseEstimation::~poseEstimation()
{
    bestPoses.clear();
	clusters.clear();
}

void poseEstimation::extractReferencePointsRandom(int & numberOfPoints, int & totalPoints)
{
	int random;
	//pcl::ExtractIndices<pcl::PointNormal> extract;
	std::vector<int> randomPoints;
	for(int i = 0; i < totalPoints; ++i)
	{
		randomPoints.push_back(i);
	}

	// Initialize random seed:
	srand ( time(NULL) );

	// Generate indices
	 for (int i = 0; i< numberOfPoints; ++i) 
	{	
		// Generate random index 
		random = rand()%randomPoints.size();
		referencePointsIndices->indices.push_back(randomPoints[random]);
		randomPoints.erase(randomPoints.begin() + random);
	}
}

void poseEstimation::extractReferencePointsUniform(float & referencePointsPercentage, int & totalPoints)
{
	unsigned int step=static_cast<unsigned int>(round(1.0/referencePointsPercentage));
	//pcl::ExtractIndices<pcl::PointNormal> extract;
	for(int i = 0; i < totalPoints; i+=step)
	{
		referencePointsIndices->indices.push_back(i);
	}
}

float poseEstimation::positionDistance(const pose & bestPose, const pose & bestCentroid)
{
	float _distance;

	// Compute distance between translations
	_distance=(bestPose.transform.translation.x() - bestCentroid.transform.translation.x()) *
		  (bestPose.transform.translation.x() - bestCentroid.transform.translation.x()) +

		  (bestPose.transform.translation.y() - bestCentroid.transform.translation.y()) *
		  (bestPose.transform.translation.y() - bestCentroid.transform.translation.y()) +

		  (bestPose.transform.translation.z() - bestCentroid.transform.translation.z()) *
		  (bestPose.transform.translation.z() - bestCentroid.transform.translation.z()); 

	return _distance; 
}

float poseEstimation::positionDistance(const cluster & c1, const cluster & c2)
{
	float _distance;

	// Compute distance between translations
	_distance=	  (c1.meanPose.transform.translation.x() - c2.meanPose.transform.translation.x()) *
		  	  (c1.meanPose.transform.translation.x() - c2.meanPose.transform.translation.x()) +

		  	  (c1.meanPose.transform.translation.y() - c2.meanPose.transform.translation.y()) *
		  	  (c1.meanPose.transform.translation.y() - c2.meanPose.transform.translation.y()) +

		  	  (c1.meanPose.transform.translation.z() - c2.meanPose.transform.translation.z()) *
		  	  (c1.meanPose.transform.translation.z() - c2.meanPose.transform.translation.z()); 

	return _distance; 
}

float poseEstimation::orientationDistance(pose & bestPose, const pose & bestCentroid)
{
	float _distance;

	// Compute distance between rotations
	_distance=(bestPose.transform.rotation.w() * bestCentroid.transform.rotation.w()) + 
		  (bestPose.transform.rotation.x() * bestCentroid.transform.rotation.x()) +
		  (bestPose.transform.rotation.y() * bestCentroid.transform.rotation.y()) +
		  (bestPose.transform.rotation.z() * bestCentroid.transform.rotation.z());

	//if(acos(_distance)>(PI-acos(poseAngleStepCos)) ) 
	if(_distance < (-pointPair::angleStepCos))
	{
		//std::cout << "before change: " << 2*acos(_distance)*RAD_TO_DEG << std::endl;
		// Compute distance between rotations
		Eigen::Quaternion<float> q2(-bestPose.transform.rotation.w(),-bestPose.transform.rotation.x(),-bestPose.transform.rotation.y(),-bestPose.transform.rotation.z());

		bestPose.transform.rotation=q2;

		_distance=(bestPose.transform.rotation.w() * bestCentroid.transform.rotation.w()) + 
			  (bestPose.transform.rotation.x() * bestCentroid.transform.rotation.x()) +
			  (bestPose.transform.rotation.y() * bestCentroid.transform.rotation.y()) +
			  (bestPose.transform.rotation.z() * bestCentroid.transform.rotation.z());

		//std::cout << "after change: " << 2*acos(_distance)*RAD_TO_DEG <<  std::endl;

	}

	if(_distance >= 1.0000000000)
	{
		_distance=1.000000000;
	//	std::cout << "UPS" << std::endl;	
	}

	return _distance; 
}

float poseEstimation::orientationDistance(cluster & c1, const cluster & c2)
{

	float _distance;

	// Compute distance between rotations
	_distance=(c1.meanPose.transform.rotation.w() * c2.meanPose.transform.rotation.w()) + 
		  	  (c1.meanPose.transform.rotation.x() * c2.meanPose.transform.rotation.x()) +
		  	  (c1.meanPose.transform.rotation.y() * c2.meanPose.transform.rotation.y()) +
		  	  (c1.meanPose.transform.rotation.z() * c2.meanPose.transform.rotation.z());


	if(_distance< (-pointPair::angleStepCos) )
	{
		//std::cout << "before change: " << 2*acos(_distance)*RAD_TO_DEG << std::endl;
		// Compute distance between rotations
		Eigen::Quaternion<float> q2(-c1.meanPose.transform.rotation.w(),-c1.meanPose.transform.rotation.x(),-c1.meanPose.transform.rotation.y(),-c2.meanPose.transform.rotation.z());

		c1.meanPose.transform.rotation=q2;

		_distance=(c1.meanPose.transform.rotation.w() * c2.meanPose.transform.rotation.w()) + 
			  	  (c1.meanPose.transform.rotation.x() * c2.meanPose.transform.rotation.x()) +
			  	  (c1.meanPose.transform.rotation.y() * c2.meanPose.transform.rotation.y()) +
			  	  (c1.meanPose.transform.rotation.z() * c2.meanPose.transform.rotation.z());

		//std::cout << "after change: " << 2*acos(_distance)*RAD_TO_DEG <<  std::endl;
	}

	if(_distance >= 1.0000000000)
	{
		_distance=1.00000000;
	//	std::cout << "UPS" << std::endl;	
	}
	return _distance; 
}

