#ifndef POSE_ESTIMATION_SV
#define POSE_ESTIMATION_SV

#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "objectrecognition/pose_estimation.h"
#include "object_model.h"



class poseEstimationSV : private poseEstimation
{
	public:
      	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Typedefs
		typedef boost::shared_ptr<objectModelSV> objectModelPtr;
		typedef boost::shared_ptr<cluster> clusterPtr;
		typedef boost::shared_ptr<poseCluster> poseClusterPtr;
		typedef boost::shared_ptr<pose> posePtr;
	// Variables
		objectModelPtr model;

	// Constructors
		poseEstimationSV(objectModelPtr inputModel);

	// Destructor
		~poseEstimationSV();
	
		std::vector< cluster > poseEstimationCore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		std::vector< cluster > poseEstimationCore_openmp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		std::vector< cluster> poseEstimationCore(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
	private:
	// Methods
		//std::vector< clusterPtr > poseClustering(std::vector < posePtr > & bestPoses);
		std::vector< cluster > poseClustering(std::vector <pose> & bestPoses);
		void clusterClusters(std::vector<cluster> & clusters);
		void accumulatorToImage();

	// Variables
		image_transport::Publisher accumulator_image_pub_;

		std::vector < std::vector<int> >  accumulator;
		std::vector < std::vector < std::vector<int> > > accumulatorParallelAux;


};

#endif //#ifndef POSE_ESTIMATION_SV
