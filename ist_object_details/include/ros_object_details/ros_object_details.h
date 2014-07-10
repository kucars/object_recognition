#ifndef MATCH_ELLIPSOID
#define MATCH_ELLIPSOID


#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/point_cloud.h>

//#include <pcl/filters/crop_hull.h>

#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <ist_msgs/Object.h>

#include "object_details/object_details.h"
#include <perception_msgs/GetRefinedPointCloud.h>
#include <perception_msgs/GetObjectDetails.h>
#include <perception_msgs/SetObjectCoordinateFrame.h>
#include <perception_msgs/Object.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>
//template <class objectModelT>
//ros::Publisher hypotheses_pub_;		
class RosObjectDetails
{
	private:
    static unsigned int neighbours;
		// Object has been detected
		bool object_detected;

		// Table has been detected
		bool table_detected;

		int num_markers_published;

		// The node handler
  		ros::NodeHandle n;

		// The private node handler
  		ros::NodeHandle n_priv;

  		// Object details markers
		ros::Publisher marker_pub;

		// Object point cloud
		ros::Publisher cloud_pub;

		// Object point cloud
		ros::Publisher cloud_pub_in;

		// Object point cloud
		ros::Publisher cloud_pub_top_part;

		// Object parts marker_array
		visualization_msgs::MarkerArray marker_array;

		// Point cloud refinement service
		ros::ServiceServer point_cloud_refinement_service;

		// Compute object details service
		ros::ServiceServer compute_object_details_service;

		// Set coordinate frame broadcaster service
		ros::ServiceServer broadcast_object_coordinate_frame_service;

	  	// Reset point cloud service
		ros::ServiceServer cloud_reset_service;

		// Table frame name
		std::string table_frame_id;

		// Table pose in world frame
		tf::Transform world_to_table_transform;

		// Table pose in camera frame
		tf::Transform table_transform;

		// Objects coordinate frames transforms
		static std::vector<tf::Transform> object_transforms;

		tf::Transform handle_transform;

		// Table parent frame name
		std::string table_parent_frame_id;

		// Object parent frame name
		std::string object_parent_frame_id;

		// Table parent frame name
		std::string world_frame_id;

		// A tf transform listener
		tf::TransformListener listener;

		// A tf broadcaster
		tf::TransformBroadcaster br;

		tf::StampedTransform table_transform_stamped;

		tf::StampedTransform object_transform_stamped;

		tf::StampedTransform handle_transform_stamped;

		pcl::PointCloud<pcl::PointXYZINormal> rviz_point_cloud;

		static unsigned int object_id_count;
		static std::vector<unsigned int> object_ids;
		static std::vector<std::string> object_frames;
		/////////////
		// METHODS //
		/////////////

		//bool serviceCallback(ist_match_ellipsoid_msgs::GetEllipsoids::Request  &req, ist_match_ellipsoid_msgs::GetEllipsoids::Response &res);

		// Service for point cloud refinement
		bool refinePointCloudServiceCallback(perception_msgs::GetRefinedPointCloud::Request  &req, perception_msgs::GetRefinedPointCloud::Response &res);

		// Service for object details computation
		bool computeObjectDetailsServiceCallback(perception_msgs::GetObjectDetails::Request &req, perception_msgs::GetObjectDetails::Response &res);

		// Service for setting the object coordinate frame
		bool setObjectCoordinateFrameServiceCallback(perception_msgs::SetObjectCoordinateFrame::Request &req, perception_msgs::SetObjectCoordinateFrame::Response &res);

		// Service for resetting cluster point clouds
		bool resetPointCloudServiceCallback(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);

		// Ellipsoid visualization number of standard deviations;
		double standard_deviations;

		// Method used to publish object details markers
		void fillObjectDetailsMarkers();

		// Method used to clear old markers
		void clearMarkers();

		//ist_msgs::Object fillObjectTypeMessage(const ObjectDetails & object);
		ist_msgs::Object fillObjectHandleProjectTypeMessage(const ObjectDetails & object);
		perception_msgs::Object fillObjectTypeMessage(const ObjectDetails & object);

		//rosEllipsoidDescription(double & _standard_deviations) : standard_deviations(_standard_deviations),current_marker_id(0)
		//{}

	public:


	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudPointXYZ;
	typedef PointCloudPointXYZ::Ptr PointCloudPointXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZI> PointCloudPointXYZI;
	typedef PointCloudPointXYZI::Ptr PointCloudPointXYZIPtr;
	typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudPointXYZINormal;
	typedef PointCloudPointXYZINormal::Ptr PointCloudPointXYZINormalPtr;
	typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
	typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;

	typedef PointCloudPointXYZINormal PointCloudIn;
	typedef PointCloudPointXYZINormalPtr PointCloudInPtr;
	typedef PointCloudPointXYZINormal::ConstPtr PointCloudInConstPtr;


		PointCloudInPtr cloudWithNormals;
		PointCloudNormalPtr cloud_normals;

    PointCloudInPtr computeNormals(PointCloudInPtr cloud);

		RosObjectDetails(ros::NodeHandle n_);
		boost::shared_ptr<ObjectDetails> object;

		void broadcastTable();
		void broadcastObject();

};

#endif //#ifndef MATCH_ELLIPSOID
