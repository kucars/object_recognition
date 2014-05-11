#ifndef POSE_ESTIMATION_SI
#define POSE_ESTIMATION_SI

#include "objectrecognition/pose_estimation.h"
#include "object_model.h"

class poseEstimationSI : private poseEstimation
{
	public:
      	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Typedefs
		typedef boost::shared_ptr<objectModelSI> objectModelPtr;
		typedef boost::shared_ptr<cluster> clusterPtr;
		typedef boost::shared_ptr<poseCluster> poseClusterPtr;
		typedef boost::shared_ptr<pose> posePtr;

	// Parameters
		static unsigned int scaleBins;
		static unsigned int scaleBinOffset;
		static unsigned int scaleMaxBin;
		static float logBase;

	// Variables
		objectModelPtr model;

	// Constructors

		// Parameters initializer
		poseEstimationSI(float _referencePointsPercentage, float _accumulatorPeakThreshold, bool _filterOn, unsigned int _scaleBins, float _logBase) : poseEstimation(_referencePointsPercentage, _accumulatorPeakThreshold, _filterOn)
		{
			scaleBins=_scaleBins;
			logBase=_logBase;

			logBaseLog=log(logBase);
			logBaseLogInverted=1.0/logBaseLog;
			if(_scaleBins%2==0.0)
				scaleBinOffset=floor(_scaleBins/2.0)-1.0; // more positive bins then negative ex: floor(10/2) -1 = 4 ... floor(9/2) = 4
			else
				scaleBinOffset=floor(_scaleBins/2.0);

			scaleMaxBin=scaleBins-1;

			//scaleLowerBound=pow(logBase,-1);
		};

		poseEstimationSI(objectModelPtr inputModel);
		~poseEstimationSI();

		std::vector< clusterPtr > poseEstimationCore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		std::vector< clusterPtr > poseEstimationCore_openmp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		std::vector<poseEstimationSI::clusterPtr> poseEstimationCore(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

	private:
	// Methods
		std::vector< clusterPtr > poseClustering(std::vector < posePtr > & bestPoses);

		float scaleDistance(poseEstimation::posePtr bestPose, poseEstimation::posePtr bestCentroid);
	// Variables
		std::vector < std::vector< std::vector<int> > >  accumulator;
		std::vector < std::vector < std::vector < std::vector<int> > > > accumulatorParallelAux;

	// private parameters
		static float logBaseLog;
		static float logBaseLogInverted;
	//static float scaleLowerBound;
};

#endif //#ifndef POSE_ESTIMATION_SI
