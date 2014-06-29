#include "ros/ros.h"
#include <ros_objectrecognition/object_recognition_ros.h>
#include <tabletop_object_segmentation_online/TabletopSegmentation.h>
//#include <pcl/ros/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <perception_msgs/PoseEstimation.h>
#include <boost/bind.hpp>

#include  <tf/transform_listener.h>

#include "objectrecognition_sv/object_model.h"
#include "objectrecognition_sv/pose_estimation.h"
#include "objectrecognition/models.h"
#include <household_objects_database_msgs/DatabaseModelPoseList.h>
#include <manipulation_msgs/GraspableObjectList.h>
#include <sstream>
#include <ist_msgs/ObjectList.h>
#include <visualization_msgs/Marker.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <shape_msgs/Mesh.h>



#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

template <class objectModelT>
ros::Publisher objectRecognitionRos<objectModelT>::marker_pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr refine_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
    ROS_INFO("ENTREI");
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    ROS_INFO("ENTREI1");

    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);

    icp.setRANSACOutlierRejectionThreshold(0.003);
    icp.setMaximumIterations(1000);
    icp.align(*Final);
    ROS_INFO("ENTREI2");

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;
    return Final;
}



pcl::PointCloud<pcl::PointNormal>::Ptr refine_icp_normals(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source,
                                                  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target)
{
    ROS_INFO("ENTREI");
    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    ROS_INFO("ENTREI1");

    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    pcl::PointCloud<pcl::PointNormal>::Ptr Final(new pcl::PointCloud<pcl::PointNormal>);

    icp.setRANSACOutlierRejectionThreshold(0.003);
    icp.setMaximumIterations(1000);
    icp.align(*Final);
    ROS_INFO("ENTREI2");

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;
    return Final;
}


class PoseEstimationROS
{
    //moveit::planning_interface::MoveGroup group;
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
public:
    /////////////////////////
    // Database parameters //
    /////////////////////////

    std::string database_name;
    std::string database_host;
    std::string database_port;
    std::string database_user;
    std::string database_pass;

    ////////////////////////////
    // Data in/out parameters //
    ////////////////////////////

    std::string processing_frame;
    std::string models_database_path;
    std::string models_database_file;
    std::string models_specification_path;
    std::string models_specification_file;

    ////////////////////////////
    // Operational parameters //
    ////////////////////////////

    int angle_bins;
    int distanceBins;
    double referencePointsPercentage;
    double pre_downsampling_step;
    double accumulatorPeakThreshold;
    double radius;
    double neighbours;
    bool radius_search;
    bool filter_on;
    int hypotheses_per_model;

    ros::ServiceServer service;

    tf::TransformListener listener;
    ros::NodeHandle n;
    ros::NodeHandle n_priv;

    // models object
    models<objectModelSV> models_library;

    std::vector< boost::shared_ptr<poseEstimationSV> > poseEstimators;

    PoseEstimationROS(ros::NodeHandle & n_, ros::NodeHandle & n_priv_) : n_priv(n_priv_), n(n_)//, group("arm")
    {
        ////////////////////////////
        // Data in/out parameters //
        ////////////////////////////

        // Working frame
        n_priv.param<std::string>("processing_frame", processing_frame, "/camera_link");

        // Models Descriptors library path
        n_priv.param<std::string>("models_database_path", models_database_path, "/home/");

        // Models Descriptors library file
        n_priv.param<std::string>("models_database_file", models_database_file, "file.test");

        //Models specification path
        n_priv.param<std::string>("models_specification_path", models_specification_path, "/filepath/");

        // Models info xml parser
        n_priv.param<std::string>("models_specification_file", models_specification_file, "file.xml");

        n_priv.param<std::string>("database_name", database_name, "household_objects-0.2");
        n_priv.param<std::string>("database_host", database_host, "localhost");
        n_priv.param<std::string>("database_port", database_port, "5432");
        n_priv.param<std::string>("database_user", database_user, "willow");
        n_priv.param<std::string>("database_pass", database_pass, "willow");

        n_priv.param<int>("angle_bins", angle_bins, 15);
        n_priv.param<int>("distance_bins", distanceBins, 20);
        n_priv.param<double>("reference_points_percentage",referencePointsPercentage, 0.50);
        n_priv.param<double>("pre_downsampling_step",pre_downsampling_step, 0);
        n_priv.param<double>("accumulatorPeakThreshold",accumulatorPeakThreshold, 0.65);
        n_priv.param<double>("radius",radius, 0.04);
        n_priv.param<double>("neighbours",neighbours, 50);
        n_priv.param<bool>("radius_search",radius_search, true);
        n_priv.param<bool>("filter_on",filter_on, false);
        n_priv.param<int>("hypotheses_per_model", hypotheses_per_model,200);

        ///////////
        // train //
        ///////////

        // Change models parameters
        objectModelSV(angle_bins,distanceBins,radius,neighbours,radius_search);


        //        ROS_INFO_STREAM(" FILE:" << (models_database_path+models_database_file));

        // Load models on the household objects database, to the "models_library" object
        models_library.loadModels(true, 1.0, n, database_name, database_host, database_port, database_user, database_pass, (models_specification_path+models_specification_file), (models_database_path+models_database_file), processing_frame);

        // Change pose estimation parameters
        poseEstimation(referencePointsPercentage,accumulatorPeakThreshold,filter_on);

        // Create pose estimation objects

        for(size_t i=0; i < models_library.objectModels.size(); i++)
            poseEstimators.push_back(boost::shared_ptr<poseEstimationSV> (new poseEstimationSV(models_library.objectModels[i]) ) );


        objectRecognitionRos<objectModelSV>::marker_pub = n.advertise<visualization_msgs::Marker>("detector_markers_out", 1);
        service = n.advertiseService("object_recognition_pose_estimation", &PoseEstimationROS::recogntion_pose_estimation, this);

        point_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointNormal> >("teste",2);
        //pcl::tracking::ParticleFilterTracker<pcl::PointXYZ> tracker;
    }

    //////////////////////
    // Service callback //
    //////////////////////

    bool recogntion_pose_estimation(perception_msgs::PoseEstimation::Request  &req,
                                    perception_msgs::PoseEstimation::Response &res)
    {
        household_objects_database_msgs::DatabaseModelPoseList pose_list;

        pcl::PointXYZ min_pt, max_pt;
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clouds;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

        int bestModelIndex;
        int bestHypothesisIndex;
        float bestHypothesisVotes;
        int region_id=0;
        if(req.cluster_list.size()==0)
        {
            ROS_INFO("No detections returned by tabletop object detector node.");
            return false;
        }

        //////////////////////
        // FOR EACH CLUSTER //
        //////////////////////

        ist_msgs::ObjectList object_list;

        manipulation_msgs::GraspableObjectList graspable_object_list;

        int cluster_id=-1;
        BOOST_FOREACH (const sensor_msgs::PointCloud& clus, req.cluster_list)
        {

            ++cluster_id;
            //msg_bottomup.table=req.table;
            ROS_INFO("FRAME_TABLE:%f,%f,%f", req.table.pose.pose.position.x,req.table.pose.pose.position.y,req.table.pose.pose.position.z);

            sensor_msgs::PointCloud clusterProcessingFrame;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sceneClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);

            /////////////////////////////////////////////////
            // convert cluster to desired processing frame //
            /////////////////////////////////////////////////

            try
            {
                ros::Time now = ros::Time::now();
                listener.waitForTransform   (processing_frame, clus.header.frame_id, now, ros::Duration(1.0));
                listener.transformPointCloud(processing_frame, clus, clusterProcessingFrame);
                //listener.transformPointCloud(processing_frame, now, clus, processing_frame, clusterProcessingFrame);

                ROS_INFO("Cluster point cloud transformed from frame %s into frame %s", clus.header.frame_id.c_str(), processing_frame.c_str());
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Failed to transform cloud from frame %s into frame %s     ", clus.header.frame_id.c_str(), processing_frame.c_str());
                return -1;
            }

            manipulation_msgs::GraspableObject graspable_object;
            graspable_object.reference_frame_id=processing_frame;
            graspable_object.cluster.header.seq=req.cluster_list[0].header.seq;
            graspable_object.cluster.header.frame_id=processing_frame;
            graspable_object.collision_name=convertInt(cluster_id);
            graspable_object.cluster=clusterProcessingFrame;
            sensor_msgs::PointCloud2 clusterProcessingFramePCL2;
            sensor_msgs::convertPointCloudToPointCloud2(clusterProcessingFrame,clusterProcessingFramePCL2); // this cloud is streamed in the output message
            graspable_object.region.cloud=clusterProcessingFramePCL2;


            // Get cluster region bounding box
            pcl::getMinMax3D(*sceneClusterCloud, min_pt, max_pt);
            //ROS_INFO("_minimumPoint.x: %f_maximumPoint.x: %f _minimumPoint.y: %f _maximumPoint.y: %f _minimumPoint.z: %f _maximumPoint.z: %f",min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);

            graspable_object.region.roi_box_dims.x=max_pt.x-min_pt.x;
            graspable_object.region.roi_box_dims.y=max_pt.y-min_pt.y;
            graspable_object.region.roi_box_dims.z=max_pt.z-min_pt.z;
            graspable_object.region.roi_box_pose.header=graspable_object.cluster.header;
            graspable_object.region.roi_box_pose.pose.position.x=max_pt.x - graspable_object.region.roi_box_dims.x/2.0;
            graspable_object.region.roi_box_pose.pose.position.y=max_pt.y - graspable_object.region.roi_box_dims.y/2.0;
            graspable_object.region.roi_box_pose.pose.position.z=max_pt.z - graspable_object.region.roi_box_dims.z/2.0;

            Hypotheses hypotheses;
            pcl::fromROSMsg(clusterProcessingFramePCL2, *sceneClusterCloud);
            for(unsigned int j=0;j<poseEstimators.size(); ++j)
            {
                //ROS_INFO("Compute hypothesis for model %d...", j);
                hypotheses.push_back(poseEstimators[j]->poseEstimationCore(sceneClusterCloud));
                //ROS_INFO("HYPOTHESIS NUMBER FOR MODEL %d: %d",j,hypotheses[j].size());
                //ROS_INFO("Done");
            }

            bestModelIndex=0;
            bestHypothesisIndex=0;
            bestHypothesisVotes=0;

            for(size_t m = 0; m < hypotheses.size(); ++m)
            {
                for(size_t p = 0; p < hypotheses[m].size(); ++p)
                {
                    household_objects_database_msgs::DatabaseModelPose pose_msg;
                    pose_msg.pose.header=req.cluster_list[0].header;
                    pose_msg.confidence=hypotheses[m][p].meanPose.votes;
                    pose_msg.pose.pose.position.x=hypotheses[m][p].meanPose.transform.translation.x();
                    pose_msg.pose.pose.position.y=hypotheses[m][p].meanPose.transform.translation.y();
                    pose_msg.pose.pose.position.z=hypotheses[m][p].meanPose.transform.translation.z();

                    pose_msg.pose.pose.orientation.w=hypotheses[m][p].meanPose.transform.rotation.w();
                    pose_msg.pose.pose.orientation.z=hypotheses[m][p].meanPose.transform.rotation.z();
                    pose_msg.pose.pose.orientation.y=hypotheses[m][p].meanPose.transform.rotation.y();
                    pose_msg.pose.pose.orientation.x=hypotheses[m][p].meanPose.transform.rotation.x();

                    pose_msg.model_id=m+1;

                    graspable_object.potential_models.push_back(pose_msg);
                    //std::cout << hypotheses[m][p].meanPose.votes << std::endl;
                }

                if(hypotheses[m][0].meanPose.votes>bestHypothesisVotes)
                {
                    bestHypothesisVotes=hypotheses[m][0].meanPose.votes;
                    bestModelIndex=m;
                }

                //models_library.objectModels[m]->modelMesh;

            }

            if(bestHypothesisVotes==0)
            {
                ROS_INFO("No hypotheses!");
                hypotheses.clear();
                continue;
            }

            ist_msgs::Object object;
            object.state.graspable_object=graspable_object;
            object_list.objects.push_back(object);

            // Transform model cloud
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudOut(new pcl::PointCloud<pcl::PointNormal>);
            ROS_INFO("Best hypothesis: modelIndex: %d clusterIndex:%d votes: %f", bestModelIndex, bestHypothesisIndex, hypotheses[bestModelIndex][bestHypothesisIndex].meanPose.votes);
            pcl::transformPointCloud(*(poseEstimators[bestModelIndex]->model->modelCloud), *cloudOut, hypotheses[bestModelIndex][bestHypothesisIndex].meanPose.getTransformation());

            sensor_msgs::PointCloud2 cloudOutMSG;
            pcl::toROSMsg(*cloudOut,cloudOutMSG);

            graspable_object.region.cloud=cloudOutMSG;
            graspable_object.region.cloud.header.frame_id=processing_frame;
            graspable_object_list.graspable_objects.push_back(graspable_object);

            clouds.push_back(cloudOut);
            clusters.push_back(sceneClusterCloud);

            // Getting Basic Information
            // ^^^^^^^^^^^^^^^^^^^^^^^^^
            //
            // We can print the name of the reference frame for this robot.

            //group.setPoseReferenceFrame("base_link");
            //group.setEndEffectorLink("end_effector");


            // Adding/Removing Objects and Attaching/Detaching Objects
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            // First, we will define the collision object message.
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = processing_frame;//group.getPlanningFrame();

            collision_object.id = "object_1";

            Eigen::Affine3f pose_eigen= hypotheses[bestModelIndex][bestHypothesisIndex].meanPose.getTransformation();
            geometry_msgs::Pose object_pose;
            tf::poseEigenToMsg((Eigen::Affine3d)pose_eigen,object_pose);
            //Eigen::Affine3f transf=hypotheses[0][0].meanPose.getTransformation();
            //tf::poseEigenToMsg((Eigen::Affine3d) transf,object_pose);
            //shape_msgs::MeshPtr mesh_ptr = models_library.objectModels[0]->modelMesh;
            collision_object.meshes.push_back(*(models_library.objectModels[0]->modelMesh));
            collision_object.mesh_poses.push_back(object_pose);
            collision_object.operation = collision_object.ADD;

            std::vector<moveit_msgs::CollisionObject> collision_objects;
            collision_objects.push_back(collision_object);

            // Now, let's add the collision object into the world
            //ROS_INFO("Add an object into the world");
            //planning_scene_interface.addCollisionObjects(collision_objects);
        }

        ///////////////////////
        // Visualize results //
        ///////////////////////

        if(!clouds.empty())
        {
            ROS_INFO("CLOUDS NOT EMPTY");
            objectRecognitionRos<objectModelSV>::visualize<pcl::PointNormal>(clouds,n,processing_frame,2,2,"detection",0.01,1.0);
        }

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (clusters[0]);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normals);

        ROS_INFO_STREAM("PROCESSING_FRAME:" << processing_frame);
        /*pcl::PointCloud<pcl::PointNormal>::Ptr cluster_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*clusters[0], *cloud_normals, *cluster_with_normals);
        pcl::PointCloud<pcl::PointNormal>::Ptr refined_point_cloud=refine_icp_normals(clouds[0], cluster_with_normals);
        refined_point_cloud->header.frame_id=processing_frame;
        point_cloud_pub.publish(refined_point_cloud);*/

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_test(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i =0; i< clouds[0]->size(); ++i)
        {
            pcl::PointXYZ point(clouds[0]->points[i].x,clouds[0]->points[i].y,clouds[0]->points[i].z);
            point_cloud_test->points.push_back(point);
        }

        //Downsample cluster...
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZ> ());
        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (clusters[0]);
        sor.setLeafSize (models_library.objectModels[0]->distanceStep,
                         models_library.objectModels[0]->distanceStep,
                         models_library.objectModels[0]->distanceStep);

        sor.filter (*cloudDownsampled);

        pcl::PointCloud<pcl::PointXYZ>::Ptr refined_point_cloud=refine_icp(point_cloud_test, cloudDownsampled);
        refined_point_cloud->header.frame_id=processing_frame;
        point_cloud_pub.publish(refined_point_cloud);

        res.object_list=graspable_object_list;

        return true;
    }

private:

    std::string convertInt(int number)
    {
        std::stringstream ss;//create a stringstream
        ss << number;//add number to the stream
        return ss.str();//return a string with the contents of the stream
    }

    typedef std::vector< std::vector<cluster> > Hypotheses;
    typedef boost::function<bool(perception_msgs::PoseEstimation::Request&, perception_msgs::PoseEstimation::Response&)> recogntion_pose_estimation_callback;

    ros::Publisher point_cloud_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recognition_pose_estimation_server");
    ros::NodeHandle n_priv("~");
    ros::NodeHandle n;

    PoseEstimationROS pose_estimation(n, n_priv);

    ROS_INFO("Ready to do object recognition and pose estimation.");

    ros::spin();

    return 0;
}












