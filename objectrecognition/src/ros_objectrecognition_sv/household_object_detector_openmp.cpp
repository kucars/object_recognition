#include <ros_objectrecognition/object_recognition_ros.h>
#include <ros/ros.h>

#include <tabletop_object_detector/TabletopSegmentation.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include  <tf/transform_listener.h>

#include <bottomup_msgs/Hypothesis.h>
#include <bottomup_msgs/Region.h>
#include <bottomup_msgs/bottomup_msg.h>
#include <bottomup_msgs/Object.h>


#include "objectrecognition_sv/object_model.h"
#include "objectrecognition_sv/pose_estimation.h"
#include "objectrecognition/models.h"

tabletop_object_detector::TabletopSegmentation srv;

ros::ServiceClient tabletop_segmentation_client_;
		
bottomup_msgs::bottomup_msg msg_bottomup;	

template <class objectModelT>
ros::Publisher objectRecognitionRos<objectModelT>::marker_pub;
ros::Publisher hypotheses_pub_;		


double fps;
clock_t time1;
void tic(){ time1=clock(); }
void tac(){ fps = (double)(clock()-time1)/(double)CLOCKS_PER_SEC; 		ROS_INFO("%3.0fms (%.1ffps)\n", fps*1000, 1/fps); }


typedef std::vector< std::vector<boost::shared_ptr<cluster> > > Hypotheses;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "threeD_object_recognition");
	ros::NodeHandle n("~");

 	// A tf transform listener
  	tf::TransformListener listener;

	int angleBins;
	int distanceBins;
	int hypotheses_per_model;
	double referencePointsPercentage;
	double pre_downsampling_step;
	double accumulatorPeakThreshold;
	double radius;
	double neighbours;
	bool radius_search;
	bool filter_on;

	std::string processing_frame;
	std::string descriptorsLibraryFile;
	std::string inputXmlFile;
	std::string descriptorsLibraryPath;
	std::string inputXmlPath;

	std::string database_name;
	std::string database_host;
	std::string database_port;
	std::string database_user;
	std::string database_pass;

    ////////////////////////////
    // Operational parameters //
    ////////////////////////////

    n.param<int>("angle_bins", angleBins, 15);
    n.param<int>("distance_bins", distanceBins, 20);
    n.param<double>("reference_points_percentage",referencePointsPercentage, 0.50);
    n.param<double>("pre_downsampling_step",pre_downsampling_step, 0);
    n.param<double>("accumulatorPeakThreshold",accumulatorPeakThreshold, 0.65);
    n.param<double>("radius",radius, 0.04);
    n.param<double>("neighbours",neighbours, 50);
    n.param<bool>("radius_search",radius_search, true);
    n.param<bool>("filter_on",filter_on, false);

    ////////////////////////////
    // Data in/out parameters //
    ////////////////////////////

   // Working frame
   n.param<std::string>("processing_frame", processing_frame, "/openni_rgb_optical_frame");
   ROS_INFO("processing_frame: %s", processing_frame.c_str());

   // Models Descriptors library path
   n.param<std::string>("models_database_path", descriptorsLibraryPath, "/home/");
   ROS_INFO("models_database_path: %s", descriptorsLibraryPath.c_str());

   // Models Descriptors library file
   n.param<std::string>("models_database_file", descriptorsLibraryFile, "file.test");
   ROS_INFO("models_database_file: %s", descriptorsLibraryFile.c_str());

   //Models specification path
   n.param<std::string>("models_specification_path", inputXmlPath, "/filepath/");
   ROS_INFO("models_specification_path: %s", inputXmlPath.c_str());

   // Models info xml parser
   n.param<std::string>("models_specification_file", inputXmlFile, "file.xml");
   ROS_INFO("models_specification_file: %s", inputXmlFile.c_str());

   // Database parameters
   n.param<std::string>("database_name", database_name, "household_objects-0.2");
   n.param<std::string>("database_host", database_host, "localhost");
   n.param<std::string>("database_port", database_port, "5432");
   n.param<std::string>("database_user", database_user, "willow");
   n.param<std::string>("database_pass", database_pass, "willow");

   n.param<int>("hypotheses_per_model", hypotheses_per_model,200);

   //std::cout << "host:"<< database_host << std::endl;
   //std::cout << "accumulator:"<< accumulatorPeakThreshold << std::endl;
   int bestModelIndex;
   int bestHypothesisIndex;
   float bestHypothesisVotes;
   pcl::PointXYZ min_pt, max_pt;
	
   Hypotheses hypotheses;

	// Segmentation service
	tabletop_segmentation_client_ = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation");


	// Markers out topic
 	objectRecognitionRos<objectModelSV>::marker_pub = n.advertise<visualization_msgs::Marker>("detector_markers_out", 1);


	// Hypothesis out topic
	hypotheses_pub_ = n.advertise<bottomup_msgs::bottomup_msg>("bottomup_detector_hypotheses_out", 1);

	///////////
	// train //
	///////////

	// Change models parameters
	objectModelSV(angleBins,distanceBins,radius,neighbours,radius_search);

	// Create models object
	models<objectModelSV> modelsLibrary;
	std::cout << " FILE:" << (inputXmlPath+descriptorsLibraryFile) << std::endl;
	// Load models on the household objects database, to the "modelsLibrary" object
	modelsLibrary.loadModels(true, 1.0, n, database_name, database_host, database_port, database_user, database_pass, (inputXmlPath+inputXmlFile), (descriptorsLibraryPath+descriptorsLibraryFile), processing_frame);

	// Change pose estimation parameters
	poseEstimation(referencePointsPercentage,accumulatorPeakThreshold,filter_on);

	// Create pose estimation objects
	std::vector< boost::shared_ptr<poseEstimationSV> > poseEstimators;
	for(size_t i=0; i < modelsLibrary.objectModels.size(); i++)
		poseEstimators.push_back(boost::shared_ptr<poseEstimationSV> (new poseEstimationSV(modelsLibrary.objectModels[i]) ) );

	msg_bottomup.hypotheses_per_object=hypotheses_per_model;
	msg_bottomup.db_objects=modelsLibrary.objectModels.size();
	msg_bottomup.db_poses=angleBins*angleBins*angleBins;
	//msg_bottomup.background_image=NULL;


	////////////
	// detect //
	////////////

	while(ros::ok())
	{
		tic();
		// Segmentation (call tabletop service)
		if(tabletop_segmentation_client_.call(srv))
  		{
			std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;


			int region_id=0;
			if(srv.response.clusters.size()==0)
			{	
				ROS_INFO("No detections returned by tabletop object detector node.");
				continue;
			}
			BOOST_FOREACH (const sensor_msgs::PointCloud& clus, srv.response.clusters)
			{
				msg_bottomup.header.seq=srv.response.clusters[0].header.seq; // BUG AQUI: RESOLVER ISTO
				msg_bottomup.header.stamp=srv.response.clusters[0].header.stamp;
				msg_bottomup.header.frame_id=processing_frame;
				msg_bottomup.table=srv.response.table;
				ROS_INFO("FRAME_TABLE:%f,%f,%f", srv.response.table.pose.pose.position.x,srv.response.table.pose.pose.position.y,srv.response.table.pose.pose.position.z);

				cout << endl;
				cout << endl;
				cout << endl;
    			sensor_msgs::PointCloud clusterProcessingFrame;
				pcl::PointCloud<pcl::PointXYZ>::Ptr sceneClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);

				/////////////////////////////////
				// convert to processing frame //
				/////////////////////////////////

				try
				{
					listener.transformPointCloud(processing_frame, clus, clusterProcessingFrame); 
					ROS_INFO("Cluster point cloud transformed from frame %s into frame %s", clus.header.frame_id.c_str(), processing_frame.c_str()); 
				}
				catch (tf::TransformException &ex)
				{
					ROS_ERROR("Failed to transform cloud from frame %s into frame %s", clus.header.frame_id.c_str(), 
					processing_frame.c_str());

				        return -1;
				}

				sensor_msgs::PointCloud2 convertedCloud;
				sensor_msgs::convertPointCloudToPointCloud2(clusterProcessingFrame,convertedCloud); // this cloud is streamed in the output message

  				pcl::fromROSMsg(convertedCloud, *sceneClusterCloud);

				tic(); 
				for(unsigned int j=0;j<poseEstimators.size(); ++j)								
				{
					//ROS_INFO("Compute hypothesis for model %d...", j);
					hypotheses.push_back(poseEstimators[j]->poseEstimationCore_openmp(sceneClusterCloud));
					//ROS_INFO("HYPOTHESIS NUMBER FOR MODEL %d: %d",j,hypotheses[j].size());
					ROS_INFO("Done");
				}
				tac();	

				/////////////////////////
				// Fill region message //
				/////////////////////////

				// Get cluster region bounding box
				pcl::getMinMax3D(*sceneClusterCloud, min_pt, max_pt);
				ROS_INFO("_minimumPoint.x: %f_maximumPoint.x: %f _minimumPoint.y: %f _maximumPoint.y: %f _minimumPoint.z: %f _maximumPoint.z: %f",min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);

				///////////////////////////////////				
				// Choose object with more votes //
				///////////////////////////////////

				std::vector<float> objectVotes;
				objectVotes.resize(hypotheses.size());
				for(size_t m = 0; m < hypotheses.size(); ++m)
				{
					objectVotes[m]=0;
					for(size_t p = 0; p < hypotheses[m].size(); ++p)
					{
						objectVotes[m]+=hypotheses[m][p]->meanPose->votes;
					}
				}

				////////////////////////////////////				
				// Choose cluster with more votes //
				////////////////////////////////////

				bestModelIndex=0;
				bestHypothesisIndex=0;
				bestHypothesisVotes=0;

				for(size_t m = 0; m < hypotheses.size(); ++m)
				{
					if(hypotheses[m][0]->meanPose->votes>bestHypothesisVotes)
					{
						bestHypothesisVotes=hypotheses[m][0]->meanPose->votes;
						bestModelIndex=m;
						//bestHypothesisIndex=0;
					}
				}

				if(bestHypothesisVotes==0)
				{	
					ROS_INFO("No hypotheses!");
					hypotheses.clear();
					continue;
				}
				// Store region message
				//std::cout << "bestModelIndex:" << bestModelIndex << " bestHypothesisIndex:" << bestHypothesisVotes << " bestHypothesisVotes:" << bestHypothesisVotes << std::endl;
				msg_bottomup.Region.push_back(objectRecognitionRos<objectModelSV>::fillRegionMsg<cluster>(region_id++, hypotheses, modelsLibrary, hypotheses_per_model, angleBins, min_pt, max_pt, processing_frame));


				// Transform model cloud
				pcl::PointCloud<pcl::PointNormal>::Ptr cloudOut(new pcl::PointCloud<pcl::PointNormal>);
				ROS_INFO("Best hypothesis: modelIndex: %d clusterIndex:%d votes: %f", bestModelIndex, bestHypothesisIndex, hypotheses[bestModelIndex][bestHypothesisIndex]->meanPose->votes);
				pcl::transformPointCloud(*(poseEstimators[bestModelIndex]->model->modelCloud), *cloudOut, hypotheses[bestModelIndex][bestHypothesisIndex]->meanPose->getTransformation());

				clouds.push_back(cloudOut);

				hypotheses.clear();
			}

			ROS_INFO("Publish results...");
			hypotheses_pub_.publish(msg_bottomup);
			msg_bottomup.Region.clear();
			ROS_INFO("Done");
			if(srv.response.clusters.size()>0)
				if(!clouds.empty())
					objectRecognitionRos<objectModelSV>::visualize<pcl::PointNormal>(clouds,n,processing_frame,0,0,"detection",0.005,1.0);
		}
		else
  		{
    			ROS_ERROR("Failed to call service tabletop_segmentation");
  		}
		tac();
	}
}

