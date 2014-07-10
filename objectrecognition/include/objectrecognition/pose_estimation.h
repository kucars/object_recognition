#ifndef POSE_ESTIMATION
#define POSE_ESTIMATION

#include <vector>
#include <omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

//#include <pcl/common/transforms.h>

#include <time.h>
#include <Eigen/StdVector>

#include <boost/math/special_functions/round.hpp>

#include <math.h>

#include "object_model.h"
#include "objectrecognition/models.h"

class poseEstimation
{
	public:
      	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Typedefs
		typedef boost::shared_ptr<cluster> clusterPtr;
		typedef boost::shared_ptr<poseCluster> poseClusterPtr;
		typedef boost::shared_ptr<pose> posePtr;
	// Parameters
		static float referencePointsPercentage;
		static float accumulatorPeakThreshold;
		static bool filterOn;

	// Constructors

		// Parameters initializer
		poseEstimation(float _referencePointsPercentage, float _accumulatorPeakThreshold, bool _filterOn)
		{
			referencePointsPercentage=_referencePointsPercentage;
			accumulatorPeakThreshold=_accumulatorPeakThreshold;
			filterOn=_filterOn;
		}

		poseEstimation();

	// Destructor
		~poseEstimation();
	

		//virtual std::vector< clusterPtr > poseEstimationCore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)=0;
		//virtual std::vector< clusterPtr > poseEstimationCore_openmp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)=0;
		//virtual std::vector<poseEstimation::clusterPtr> poseEstimationCore(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)=0;
		std::vector < cluster > clusters;

	private:
	// Methods
		friend class poseEstimationSV;
		friend class poseEstimationSI;

		float positionDistance(const pose & bestPose, const pose & bestCentroid);
		float positionDistance(const cluster & c1, const cluster & c2);
		float orientationDistance(pose & bestPose, const pose & bestCentroid);
		float orientationDistance(cluster & c1,  const cluster & c2);


		void extractReferencePointsRandom(int & numberOfPoints, int & totalPoints);
		void extractReferencePointsUniform(float & referencePointsPercentage, int & totalPoints);
	// Variables
		//std::vector < std::vector<int> >  accumulator;
		//std::vector < std::vector < std::vector<int> > > accumulatorParallelAux;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownsampled;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormalsDownSampled;
		pcl::PointIndices::Ptr referencePointsIndices;
		//std::vector < posePtr > bestPoses;
	        std::vector < pose > bestPoses;
};

#endif //#ifndef POSE_ESTIMATION
