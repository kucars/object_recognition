#include <ros_objectrecognition/object_recognition_ros.h>
#include <ros/ros.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/bind.hpp>

#include <bottomup_msgs/Hypothesis.h>
#include <bottomup_msgs/Region.h>
#include <bottomup_msgs/bottomup_msg.h>
#include <bottomup_msgs/Object.h>

#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

//#include <orca_proxy/sensor_object_set.h>

#include "objectrecognition_sv/object_model.h"
#include "objectrecognition_sv/pose_estimation.h"

template <class objectModelT>
ros::Publisher objectRecognitionRos<objectModelT>::marker_pub; // markers out

double fps;
clock_t time1;
void tic(){ time1=clock(); }
void tac(){ fps = (double)(clock()-time1)/(double)CLOCKS_PER_SEC; 		ROS_INFO("%3.0fms (%.1ffps)\n", fps*1000, 1/fps); }
	
// Create models object
models<objectModelSV> modelsLibrary;

void bottomupListenerCallback(const bottomup_msgs::bottomup_msgConstPtr & msg, const ros::NodeHandle & n, const std::string & _processing_frame)
{
	ROS_INFO("NOVA MEDIDA DO FILTRO!");
 	// A tf transform listener
  	tf::TransformListener listener;

	//Eigen::eigen2_Transform3d transformation;

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> graspablePointsClouds;
	double confidenceMax; 
	int besthypothesis, bestobject;
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	/////////////////////
	// For each region //
	/////////////////////

	for(size_t i=0 ; i<msg->Region.size(); i++)
	{	

		//////////////////////
		// choose best pose //
		//////////////////////

		confidenceMax = FLT_MIN;
		besthypothesis = 0; 
		//bestobject = msg->Region[i].hypothesis[0].obj_id;//, 
		bestobject = 1;
		// get the best hypothesis for current region
		for(size_t hypothesis=0 ; hypothesis<msg->Region[i].hypothesis.size() ; ++hypothesis)
		{
			if(msg->Region[i].hypothesis[hypothesis].likelihood > confidenceMax)
			{
				confidenceMax = msg->Region[i].hypothesis[hypothesis].likelihood;
				besthypothesis = hypothesis;
				//bestobject = msg->Region[i].hypothesis[besthypothesis].obj_id; // DESCOMENTAR ISTO QUANDO ESTIVER TUDO BEM
			}
		}

		ROS_INFO("\nRegion: %d\t \n  BEST POSE: \n\tlikelihood: %f \n\tobject id: %d \n\tobject id(teste): %d\n\tobject db id: %d \n\torientation: %d->(yaw,pitch,roll)=(%.3fdeg,%.3fdeg,%.3fdeg) \n\tposition: (x,y,z)=(%.3fcm,%.3fcm,%.3fcm)\n",(int)i, msg->Region[i].hypothesis[besthypothesis].likelihood, msg->Region[i].hypothesis[besthypothesis].obj_id,bestobject, msg->Region[i].hypothesis[besthypothesis].db_id, msg->Region[i].hypothesis[besthypothesis].p , msg->Region[i].hypothesis[besthypothesis].yaw*RAD_TO_DEG,msg->Region[i].hypothesis[besthypothesis].pit*RAD_TO_DEG,msg->Region[i].hypothesis[besthypothesis].rol*RAD_TO_DEG,msg->Region[i].hypothesis[besthypothesis].x*0.1,msg->Region[i].hypothesis[besthypothesis].y*0.1,msg->Region[i].hypothesis[besthypothesis].z*0.1);

		//////////////////////////////////////
		// convert pose to processing frame //
		//////////////////////////////////////

		geometry_msgs::PoseStamped poseProcessingFrameMsg;

		try
		{
    			ros::Time now = ros::Time::now();
    			listener.waitForTransform(_processing_frame, msg->Region[i].hypothesis[besthypothesis].pose.header.frame_id, now, ros::Duration(3.0));
			listener.transformPose(_processing_frame, msg->Region[i].hypothesis[besthypothesis].pose, poseProcessingFrameMsg); 
			ROS_INFO("Pose converted from frame %s into frame %s", msg->Region[i].hypothesis[besthypothesis].pose.header.frame_id.c_str(), _processing_frame.c_str()); 


			// ACCESSING POSE
			//msg->Region[i].hypothesis[besthypothesis].pose.pose.position.x;
			//msg->Region[i].hypothesis[besthypothesis].pose.pose.orientation.w;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("Failed to transform pose from frame %s into frame %s", msg->Region[i].hypothesis[besthypothesis].pose.header.frame_id.c_str(), 
			_processing_frame.c_str());

		        return;
		}

		/////////////////////////////////
		// convert pose from msg to tf //
		/////////////////////////////////

		tf::Stamped<tf::Pose> pose;
		tf::poseStampedMsgToTF(poseProcessingFrameMsg,pose);	
  		tf::Transform transformation(pose);
		///////////////////////////////////////
		// get transformed model point cloud //
		///////////////////////////////////////

		pcl::PointCloud<pcl::PointNormal>::Ptr cloudOut(new pcl::PointCloud<pcl::PointNormal>);

		ROS_INFO_STREAM("library size: " << modelsLibrary.objectModels.size() << " best index: " << bestobject);
		pcl_ros::transformPointCloudWithNormals(*(modelsLibrary.objectModels[bestobject-1]->modelCloud), *cloudOut, transformation);
		ROS_INFO("PASSOU");

		clouds.push_back(cloudOut);
		ROS_INFO("PASSOU2");
		//////////////////////////////////////
		// get transformed graspable points //
		//////////////////////////////////////

		//pcl::PointCloud<pcl::PointNormal>::Ptr graspablePoints(new pcl::PointCloud<pcl::PointNormal>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr scenePclCloud(new pcl::PointCloud<pcl::PointXYZ>);			
  		//pcl::fromROSMsg(msg->Region[i].scene_cloud, *scenePclCloud);		
		//graspablePoints=modelsLibrary.objectModels[bestobject]->sceneGraspablePoints(transformation);
		//graspablePointsClouds.push_back(graspablePoints);
	}

	///////////////////////
	// Visualize results //
	///////////////////////

	if(!clouds.empty())
	{
		ROS_INFO("PASSOU3");
		objectRecognitionRos<objectModelSV>::visualize<pcl::PointNormal>(clouds,n,_processing_frame,2,2,"detection_filtered",0.01,1.0);
		ROS_INFO("PASSOU4");
		//objectRecognitionRos<objectModelSV>::visualize<pcl::PointNormal>(graspablePointsClouds,n,_processing_frame,1,1,"graspable_points",0.01,1.0);
	}



}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "threeD_bottomup_graspable_points");
	ros::NodeHandle n("~");
	int angleBins;
	int distanceBins;

	double referencePointsPercentage;
	double pre_downsampling_step;
	double accumulator_peak_threshold;
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

    	//initialize operational flags
    	n.param<int>("feature_angle_bins", angleBins, 30);
    	n.param<int>("feature_distance_bins", distanceBins, 20);

    	n.param<double>("reference_points_percentage",referencePointsPercentage, 0.50);
    	n.param<double>("pre_downsampling_step",pre_downsampling_step, 0);
	n.param<double>("accumulator_peak_threshold",accumulator_peak_threshold, 0.65);
    	n.param<double>("radius",radius, 0.04);
    	n.param<double>("neighbours",neighbours, 50);
    	n.param<bool>("radius_search",radius_search, true);
    	n.param<bool>("filter_on",filter_on, false);

	// Working frame
    n.param<std::string>("processing_frame", processing_frame, "/openni_rgb_optical_frame");
	ROS_INFO("processing_frame: %s", processing_frame.c_str());

	// Models Descriptors library path
    n.param<std::string>("models_database_path", descriptorsLibraryPath, "/home/icub/repositories/vislab/ros/object_recognition/3drecognition/objectrecognition/global_model_descriptors/");
	ROS_INFO("models_database_path: %s", descriptorsLibraryPath.c_str());

	// Models Descriptors library file
    n.param<std::string>("models_database_file", descriptorsLibraryFile, "ikea.test");
	ROS_INFO("models_database_file: %s", descriptorsLibraryFile.c_str());

	//Models specification path
    n.param<std::string>("models_specification_path", inputXmlPath, "/home/icub/repositories/ros/object_recognition/3drecognition/objectrecognition/config/");
	ROS_INFO("models_specification_path: %s", inputXmlPath.c_str());
	
	// Models info xml parser
    n.param<std::string>("models_specification_file", inputXmlFile, "models.xml");
	ROS_INFO("models_specification_file: %s", inputXmlFile.c_str());

	// Database parameters
    	n.param<std::string>("database_name", database_name, "household_objects-0.2");
    	n.param<std::string>("database_host", database_host, "localhost");
    	n.param<std::string>("database_port", database_port, "5432");
    	n.param<std::string>("database_user", database_user, "willow");
    	n.param<std::string>("database_pass", database_pass, "willow");

	ros::Subscriber hypotheses_sub_ = n.subscribe<bottomup_msgs::bottomup_msg>("/hypotheses_in", 2,boost::bind(&bottomupListenerCallback,_1,n,processing_frame)); // subscribe to tracker_in topic

	// Markers out topic
 	objectRecognitionRos<objectModelSV>::marker_pub = n.advertise<visualization_msgs::Marker>("markers_out", 0); // advertise visualization markers to markers_out topic

	/////////////////
	// Load Models //
	/////////////////

	
	// Change models parameters
	objectModelSV(angleBins,distanceBins,radius,neighbours,radius_search);

	std::cout << " FILE:" << (inputXmlPath+descriptorsLibraryFile) << std::endl;
	// Load models on the household objects database, to the "modelsLibrary" object
	modelsLibrary.loadModels(false, 1.0, n, database_name, database_host, database_port, database_user, database_pass, inputXmlPath+inputXmlFile, descriptorsLibraryPath+descriptorsLibraryFile, processing_frame);
	
	ros::spin();
}

