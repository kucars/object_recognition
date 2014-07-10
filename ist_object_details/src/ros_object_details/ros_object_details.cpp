#include "ros_object_details/ros_object_details.h"
#include <iostream>
#include <fstream>
double fps;
clock_t time1;
void tic(){ time1=clock(); }
double tac(){ fps = (double)(clock()-time1)/(double)CLOCKS_PER_SEC; 		ROS_INFO("%9.0fms (%.1ffps)\n", fps*1000, 1/fps); return fps*1000.0; }

std::vector<tf::Transform> RosObjectDetails::object_transforms;
std::vector<unsigned int> RosObjectDetails::object_ids;
std::vector<std::string> RosObjectDetails::object_frames;
unsigned int RosObjectDetails::object_id_count;
unsigned int RosObjectDetails::neighbours;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr cloud)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    const std::string points="points";
    const std::string normals="normals";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal	> single_color(cloud, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZINormal> (cloud, single_color, points,0);
    viewer->addPointCloudNormals<pcl::PointXYZINormal>(cloud,1,0.01,"normals", 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, points);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();
    return (viewer);
}

RosObjectDetails::RosObjectDetails(ros::NodeHandle n_) : object_detected(false), table_detected(false), n(n_), n_priv("~")
{
    cloud_normals = PointCloudNormalPtr (new PointCloudNormal);


    cloudWithNormals = PointCloudInPtr (new PointCloudIn);

    ////////////////
    // Parameters //
    ////////////////
    bool remove_outliers;
    num_markers_published=0;
    object=boost::shared_ptr<ObjectDetails> (new ObjectDetails);
    double minimum_handle_points_percentage;
    int normals_neighbours;
    double object_type_threshold;
    double object_parts_threshold;
    double inliers_circle_threshold;
    double top_height_threshold; // percentage/100 of object height

    double downsampling_step;
    double handle_distance_threshold;
    double circle_ransac_distance_threshold;
    double circle_minimum_radius;
    double circle_maximum_radius;
    double centroid_distance_threshold;
    double vertical_height_threshold;

    double hull_alpha;

    n_priv.param<bool>("remove_outliers", remove_outliers, false);
    ROS_INFO("remove outliers: %d", remove_outliers);

    n_priv.param<double>("object_type_threshold", object_type_threshold, 0.1); // 0.1=10%
    ROS_INFO("object type threshold: %f", object_type_threshold);

    n_priv.param<double>("object_parts_threshold", object_parts_threshold, 0.03);
    ROS_INFO("object parts threshold: %f", object_parts_threshold);

    n_priv.param<double>("top_height_threshold", top_height_threshold, 0.2);
    ROS_INFO("top height threshold: %f", top_height_threshold);

    n_priv.param<double>("downsampling_step", downsampling_step, 0.005);
    ROS_INFO("downsampling step: %f", downsampling_step);

    n_priv.param<double>("inliers_circle_threshold", inliers_circle_threshold, 0.9);
    ROS_INFO("inliers circle threshold: %f", inliers_circle_threshold);

    n_priv.param<double>("circle_ransac_distance_threshold", circle_ransac_distance_threshold, 0.9);
    ROS_INFO("circle ransac distance threshold: %f", circle_ransac_distance_threshold);

    n_priv.param<double>("circle_minimum_radius", circle_minimum_radius, 0.03);
    ROS_INFO("circle minimum radius: %f", circle_minimum_radius);

    n_priv.param<double>("circle_maximum_radius", circle_maximum_radius, 0.1);
    ROS_INFO("circle maximum radius: %f", circle_maximum_radius);

    n_priv.param<double>("handle_distance_threshold", handle_distance_threshold, 0.002);
    ROS_INFO("handle distance threshold: %f", handle_distance_threshold);

    n_priv.param<double>("centroid_distance_threshold", centroid_distance_threshold, 0.03);
    ROS_INFO("centroid distance threshold: %f", centroid_distance_threshold);

    n_priv.param<double>("vertical_height_threshold", vertical_height_threshold, 0.2);
    ROS_INFO("vertical height threshold: %f", vertical_height_threshold);

    n_priv.param<double>("hull_alpha", hull_alpha, 0.2);
    ROS_INFO("hull alpha: %f", hull_alpha);

    n_priv.param<double>("minimum_handle_points_percentage", minimum_handle_points_percentage, 0.05);
    ROS_INFO("minimum handle points percentage: %f", minimum_handle_points_percentage);

    n_priv.param<int>("normals_neighbours", normals_neighbours, 6);
    ROS_INFO("normals neighbours: %d", normals_neighbours);

    n_priv.param<std::string>("world_frame_id", world_frame_id, "/base_link");
    ROS_INFO("world frame id: %s", world_frame_id.c_str());

    n_priv.param<std::string>("table_frame_id", table_frame_id, "/table_frame");
    ROS_INFO("table frame id: %s", table_frame_id.c_str());

    ObjectDetails::setRemoveOutliers(remove_outliers);
    ObjectDetails::setObjectTypeThreshold(object_type_threshold);
    ObjectDetails::setObjectPartsThreshold(object_parts_threshold);
    ObjectDetails::setTopHeightThreshold(top_height_threshold);
    ObjectDetails::setDownsamplingStep(downsampling_step);
    ObjectDetails::setMinHandlePointsPercentage(minimum_handle_points_percentage);
    ObjectDetails::setInliersCircleThreshold(inliers_circle_threshold);
    ObjectDetails::setHandleDistanceThreshold(handle_distance_threshold);
    ObjectDetails::setRansacCircleThreshold(circle_ransac_distance_threshold);
    ObjectDetails::setCircleMinimumRadius(circle_minimum_radius);
    ObjectDetails::setCircleMaximumRadius(circle_maximum_radius);
    ObjectDetails::setCentroidDistanceThreshold(centroid_distance_threshold);
    ObjectDetails::setVerticalHeightThreshold(vertical_height_threshold);
    ObjectDetails::setHullAlpha(hull_alpha);
    ObjectDetails::setNeighbours((unsigned int)normals_neighbours);
    neighbours=normals_neighbours;

    //////////
    // RVIZ //
    //////////

    // Complete cloud out topic
    cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("refined_point_cloud_out", 1, true);

    marker_pub = n.advertise<visualization_msgs::MarkerArray> ("object_details_markers_out",1, true);

    //////////////
    // Services //
    //////////////

    // Point cloud refinement service
    point_cloud_refinement_service = n.advertiseService("ist_point_cloud_refinement", &RosObjectDetails::refinePointCloudServiceCallback,this);

    // Object details service
    compute_object_details_service = n.advertiseService("ist_compute_object_details", &RosObjectDetails::computeObjectDetailsServiceCallback,this);

    // Reset point cloud service
    cloud_reset_service = n.advertiseService("ist_reset_point_cloud", &RosObjectDetails::resetPointCloudServiceCallback,this);
}

bool RosObjectDetails::resetPointCloudServiceCallback(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
    ROS_INFO("Reset point cloud service called...");
    object_detected=false;
    object_id_count=0;
    object_ids.clear();
    object_frames.clear();
    object_transforms.clear();
    //clearMarkers();
    rviz_point_cloud.points.clear();
    return true;
}

/*bool RosObjectDetails::addObjectMarkers(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
    ROS_INFO("Add objects markers...");


    fillObjectDetailsMarkers();
    marker_pub.publish(marker_array);
    marker_array.markers.clear();

    return true;
}*/


bool RosObjectDetails::refinePointCloudServiceCallback(perception_msgs::GetRefinedPointCloud::Request  &req, perception_msgs::GetRefinedPointCloud::Response &res )
{
    ROS_INFO("Refine point cloud service...");

    std::cout << "size in: " << req.point_cloud.points.size() << std::endl;
    sensor_msgs::PointCloud ros_point_cloud;
    sensor_msgs::PointCloud2 ros_point_cloud2;

    table_parent_frame_id=req.table.pose.header.frame_id;
    ROS_INFO_STREAM("PARENT FRAME: " << req.table.pose.header.frame_id);
    tf::poseMsgToTF(req.table.pose.pose, table_transform);

    table_detected=true;
    broadcastTable();




    ////////////////////////////////////
    // Convert cluster to table frame //
    ////////////////////////////////////

    Eigen::Affine3d eigen_table_transform;

    // Table pose with respect to parent frame
    tf::poseMsgToEigen(req.table.pose.pose, eigen_table_transform);

    sensor_msgs::convertPointCloudToPointCloud2(req.point_cloud,ros_point_cloud2); // convert to sensor_msgs/PointCloud2

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_point_cloud2, *pcl_point_cloud_temp); // convert to sensor_msgs/PointCloud2 to pcl

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_camera(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::copyPointCloud(*pcl_point_cloud_temp, *pcl_point_cloud); // THIS IS IMPORTANT (KINECT DOESN'T HAVE INTENSITY FIELD)s

    // Convert firstm to camera frame to get normals
    tf::TransformListener listener;
    tf::StampedTransform camera_transform;

    try
    {
        ROS_INFO_STREAM("FRAME_ID:" << req.point_cloud.header.frame_id);
        listener.waitForTransform(req.point_cloud.header.frame_id, "/openni_rgb_frame", ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform(req.point_cloud.header.frame_id, "/openni_rgb_frame", ros::Time(0),  camera_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    /// Converts a tf Transform into an Eigen Affine3d
    // Transform to camera frame
    Eigen::Affine3d eigen_camera_transform;
    tf::transformTFToEigen(camera_transform, eigen_camera_transform);
    pcl::transformPointCloud(*pcl_point_cloud, *pcl_point_cloud_camera, (Eigen::Affine3f) eigen_camera_transform);


    // Compute normals
    pcl_point_cloud=computeNormals(pcl_point_cloud_camera);


    pcl::transformPointCloudWithNormals(*pcl_point_cloud, *pcl_point_cloud_camera, (Eigen::Affine3f) eigen_camera_transform.inverse());

    // PCL point cloud in table frame
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_transformed(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud(*pcl_point_cloud_camera, *pcl_point_cloud_transformed, (Eigen::Affine3f) eigen_table_transform.inverse());

    // Complete point cloud
    ROS_INFO("Complete point cloud...");
    pcl_point_cloud_transformed->header.frame_id=world_frame_id;
    //object->completeObjectPointCloud(pcl_point_cloud_transformed);

    std::ofstream outputFile;
    outputFile.open("object_completion_statistics.txt", ios::app);
    tic();
    object->completeObjectPointCloudAlternative(pcl_point_cloud_transformed);
    double time=tac();
    outputFile << "Object name: " << req.object_name.data << " points in: " << (int)req.point_cloud.points.size() << " points out: " << (int)object->complete_point_cloud->size() << "  time (ms): " << time << std::endl;
    pcl_point_cloud=object->complete_point_cloud;
    std::cout << "COMPLETE SIZE;" << object->complete_point_cloud->size() << std::endl;
    ROS_INFO("Done.");


    ///////////////////////////////////////////////
    // Convert refined cluster to original frame //
    ///////////////////////////////////////////////

    pcl::transformPointCloud(*pcl_point_cloud, *pcl_point_cloud_transformed, (Eigen::Affine3f) eigen_table_transform);
    // Convert PCL to ROS point cloud
    pcl::toROSMsg(*pcl_point_cloud_transformed,ros_point_cloud2);
    ros_point_cloud2.header.frame_id=req.point_cloud.header.frame_id;
    ros_point_cloud2.header.stamp=ros::Time::now();
    sensor_msgs::convertPointCloud2ToPointCloud(ros_point_cloud2,ros_point_cloud);
    res.point_cloud=ros_point_cloud;
    // See in rviz
    rviz_point_cloud.header.frame_id=req.table.pose.header.frame_id;
    pcl_point_cloud_transformed->header=rviz_point_cloud.header;
    rviz_point_cloud+=*pcl_point_cloud_transformed;
    cloud_pub.publish(rviz_point_cloud);

    ////////////////////////////////////////////////////
    // Convert MEGA refined cluster to original frame //
    ////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_xyz_reflection(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl_point_cloud_xyz_reflection=object->point_cloud_complete_xyz_reflect;
    pcl::transformPointCloud(*pcl_point_cloud_xyz_reflection, *pcl_point_cloud_transformed, (Eigen::Affine3f) eigen_table_transform);

    // Convert PCL to ROS point cloud
    pcl::toROSMsg(*pcl_point_cloud_transformed,ros_point_cloud2);
    ros_point_cloud2.header.frame_id=req.point_cloud.header.frame_id;
    ros_point_cloud2.header.stamp=ros::Time::now();
    sensor_msgs::convertPointCloud2ToPointCloud(ros_point_cloud2, ros_point_cloud);
    res.point_cloud_object_details=ros_point_cloud;

    std::cout << "size out: " << ros_point_cloud.points.size() << std::endl;

    return true;
}

bool RosObjectDetails::computeObjectDetailsServiceCallback(perception_msgs::GetObjectDetails::Request &req, perception_msgs::GetObjectDetails::Response &res)
{
    ROS_INFO("Compute object details service...");

    //std::cout << "size in: " << req.point_cloud.points.size() << std::endl;
    sensor_msgs::PointCloud ros_point_cloud;
    sensor_msgs::PointCloud2 ros_point_cloud2;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_world(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_object_details_world(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_point_cloud_parent(new pcl::PointCloud<pcl::PointXYZINormal>);

    ////////////////////////////////////
    // Convert cluster to world frame //
    ////////////////////////////////////

    // Get parent to world transform
    tf::StampedTransform parent_to_world_transform;
    bool transform_acquired=false;
    //ROS_INFO_STREAM("teste:"<<req.point_cloud.header.frame_id);
    //exit(-1);
    while((!transform_acquired) && ros::ok())
    {
        transform_acquired=true;
        try
        {
            ROS_INFO("Wait for transform from %s to %s", req.point_cloud.header.frame_id.c_str(), world_frame_id.c_str());
            listener.waitForTransform(world_frame_id,    req.point_cloud.header.frame_id, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform (world_frame_id,    req.point_cloud.header.frame_id, ros::Time(0), parent_to_world_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("Transform not acquired.");
            transform_acquired=false;
        }
    }

    // Transform data conversion
    // Object pose with respect to world frame
    geometry_msgs::Pose world_pose_message;
    tf::poseTFToMsg(parent_to_world_transform, world_pose_message);

    Eigen::Affine3d eigen_world_transform;
    tf::poseMsgToEigen(world_pose_message, eigen_world_transform);

    // Point cloud data conversion
    //sensor_msgs::convertPointCloudToPointCloud2(req.point_cloud, ros_point_cloud2); // convert to sensor_msgs/PointCloud2
    pcl::fromROSMsg(req.point_cloud, *pcl_point_cloud_parent); // convert to sensor_msgs/PointCloud2 to pcl

    pcl_point_cloud_world->header.frame_id=world_frame_id;
    pcl::transformPointCloud(*pcl_point_cloud_parent, *pcl_point_cloud_world, (Eigen::Affine3f) eigen_world_transform);
    pcl_point_cloud_world->header.frame_id= world_frame_id;


    // Point cloud object details data conversion
    //sensor_msgs::convertPointCloudToPointCloud2(req.point_cloud_object_details,req.point_cloud); // convert to sensor_msgs/PointCloud2
    pcl::fromROSMsg(req.point_cloud, *pcl_point_cloud_parent); // convert to sensor_msgs/PointCloud2 to pcl


    pcl_point_cloud_object_details_world->header.frame_id=world_frame_id;
    pcl::transformPointCloud(*pcl_point_cloud_parent, *pcl_point_cloud_object_details_world, (Eigen::Affine3f) eigen_world_transform);
    pcl_point_cloud_object_details_world->header.frame_id= world_frame_id;


    ////////////////////////////
    // Compute object details //
    ////////////////////////////
    //std::ofstream outputFile;
    //outputFile.open("object_details_computation_statistics.txt", ios::app);
    //tic();
    object->computeObjectDetails(pcl_point_cloud_object_details_world, pcl_point_cloud_world);


    //double time=tac();
    //outputFile << "Object name: " << req.object_name.data <<" points in: " << (int)req.point_cloud.points.size() << " time (ms): " << time << std::endl;
    ///////////////////
    // Fill response //
    ///////////////////

    res.object=fillObjectHandleProjectTypeMessage(*object);


    //////////////////////////
    // Publish rviz markers //
    //////////////////////////

    fillObjectDetailsMarkers();

    if(marker_array.markers.size()>0)
    {
        marker_pub.publish(marker_array);
        marker_array.markers.clear();
    }

    tf::Transform object_transform;

    tf::poseMsgToTF(res.object.state.graspable_object.potential_models[0].pose.pose, object_transform);
    object_transforms.push_back(object_transform);

    /*geometry_msgs::PoseStamped handle_pose;
    handle_pose.header.stamp=ros::Time::now();
    handle_pose.header.frame_id="object_1";	// Fill header (very important to know the reference coordinate frame)
    if(object->has_handle)
    {
        tf::poseEigenToMsg((Eigen::Affine3d) object->regions[3].pose, handle_pose.pose);
        tf::poseMsgToTF(handle_pose.pose, handle_transform);
    }*/


    object_parent_frame_id=res.object.state.graspable_object.potential_models[0].pose.header.frame_id;
    // Broadcast object with respect to world frame
    object_detected=true;
    broadcastObject();

    return true;
}

ObjectDetails::PointCloudInPtr RosObjectDetails::computeNormals(PointCloudInPtr cloud)
{


    //pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);

    // Create the normal estimation class, and pass the input dataset to it
    // pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne; ///VERSAO MAIS RAPIDA POIS USA multiplas threads! explorar isto
    pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    //KdTreePtr (new KdTree);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    //ne.setSearchSurface (cloud);
    //pcl::compute3DCentroid(*cloud,centroid);
    ne.setViewPoint(0,0,0);
    //ne.setViewPoint(centroid(0),centroid(1),centroid(2));
    //std::cout << centroid << std::endl;
    // Output datasets
    /*	if(radiusSearch)
    {
        // Use all neighbors in a sphere of radius 'radius'
        ne.setRadiusSearch(radius);
    }
    else*/
    {
        // Use 'neighbours' neighbors
        ne.setKSearch (neighbours);
    }
    // Compute the features
    ne.compute(*cloud_normals);


    // Concatenate point XYZ with it's normals
    pcl::concatenateFields(*cloud, *cloud_normals, *cloudWithNormals);

    return cloudWithNormals;
}


ist_msgs::Object RosObjectDetails::fillObjectHandleProjectTypeMessage(const ObjectDetails & object)
{
    object_ids.push_back(object_id_count);
    ++object_id_count;
    std::stringstream object_frame_name;
    object_frame_name << "object_";
    object_frame_name << object_id_count;
    object_frames.push_back(object_frame_name.str());

    sensor_msgs::PointCloud ros_point_cloud;
    sensor_msgs::PointCloud2 ros_point_cloud2;

    ist_msgs::Object msg;

    // ID number
    msg.object_id=object_ids.back();

    // Collision name equal to object reference frame name
    msg.collision_name=object_frames.back();

    if(object.has_handle)
    {
        msg.data.type.id=2;
        msg.data.type.type_name="container";
        msg.data.type.is_container=true;
    }
    else if(object.is_tool)
    {
        msg.data.type.id=3;
        msg.data.type.type_name="tool";
        msg.data.type.is_container=false;
    }
    else
    {
        msg.data.type.id=1;
        msg.data.type.type_name="container";
        msg.data.type.is_container=true;
    }

    // Object symmetry (discrete)
    msg.data.type.symmetry.value=object.object_symmetry_type;
    ROS_INFO("Symmetry type: %d", msg.data.type.symmetry.value);

    // Object shape type (discrete)
    msg.data.type.shape.value=object.object_shape_type;
    ROS_INFO("Shape type: %d", msg.data.type.shape.value);

    // Object size type (discrete)
    msg.data.type.size.id=object.object_parts_number;
    ROS_INFO("Object size type (discrete): %d", msg.data.type.size.id);

    // Object size type (continuous)
    msg.data.type.size.values.x=object.bounding_box.x();
    msg.data.type.size.values.y=object.bounding_box.y();
    msg.data.type.size.values.z=object.bounding_box.z();
    ROS_INFO("Object size type (continuous): (%f,%f,%f)", msg.data.type.size.values.x, msg.data.type.size.values.y, msg.data.type.size.values.z);

    // Object point cloud

    pcl::toROSMsg(*(object.centered_point_cloud),ros_point_cloud2);
    //pcl::toROSMsg(*(object.complete_point_cloud),ros_point_cloud2);

    /*for(pcl::PointCloud<pcl::PointXYZINormal>::iterator p=(object.centered_point_cloud)->begin(); p < (object.centered_point_cloud)->end(); ++p )
    {
        std::cout << "intensity: " << p->intensity << " ";
    }
    std::cout << std::endl;
    */
    sensor_msgs::convertPointCloud2ToPointCloud(ros_point_cloud2,ros_point_cloud);

    msg.state.graspable_object.cluster=ros_point_cloud;
    // Object pose with respect to corresponding reference coordinate frame
    geometry_msgs::PoseStamped object_pose_msg;
    object_pose_msg.header.stamp=ros::Time::now();
    object_pose_msg.header.frame_id=world_frame_id;	// Fill header (very important to know the reference coordinate frame)

    Eigen::Affine3f temp(object.pose.matrix());
    tf::poseEigenToMsg((Eigen::Affine3d) temp, object_pose_msg.pose);

    msg.state.graspable_object.potential_models.resize(1);
    msg.state.graspable_object.potential_models[0].pose=object_pose_msg; // relative to the object reference frame (where to store the pose relative to parent frame?)
    //ROS_INFO_STREAM("POSITION:" << msg.state.graspable_object.potential_models[0].pose.pose.position);
    msg.state.discrete_pose=object.discrete_vertical_orientation;

    for(unsigned int i=0; i<object.regions.size(); ++i)
    {
        std::cout << "OBJECT PART: " << object.regions[i].id << std::endl;
        ist_msgs::ActionablePartData actionable_part_data_msg;
        actionable_part_data_msg.part.id=object.regions[i].id; // part id
        actionable_part_data_msg.part.bounding_box.x=object.regions[i].bounding_box.x();
        actionable_part_data_msg.part.bounding_box.y=object.regions[i].bounding_box.y();
        actionable_part_data_msg.part.bounding_box.z=object.regions[i].bounding_box.z();
        actionable_part_data_msg.part.confidence=object.regions[i].likelihood;
        Eigen::Affine3f temp(object.regions[i].pose.matrix());

        tf::poseEigenToMsg((Eigen::Affine3d) temp, actionable_part_data_msg.part.pose.pose); // We consider the part pose to be relatively to object

        actionable_part_data_msg.part.pose.header.frame_id=(object_frames.back());
        actionable_part_data_msg.type.id=object.regions[i].type; // type
        msg.data.actionable_parts_data.push_back(actionable_part_data_msg);
    }

    return msg;
}

perception_msgs::Object RosObjectDetails::fillObjectTypeMessage(const ObjectDetails & object)
{
    sensor_msgs::PointCloud ros_point_cloud;
    sensor_msgs::PointCloud2 ros_point_cloud2;

    perception_msgs::Object msg;

    msg.task.id=0;
    msg.task.likelihood=1.0;

    // Fill regions
    msg.regions.resize(object.regions.size());
    for(unsigned int i=0; i<object.regions.size(); ++i)
    {
        perception_msgs::Region region_msg;
        region_msg.type=object.regions[i].type; // part id
        region_msg.bounding_box.x=object.regions[i].bounding_box.x();
        region_msg.bounding_box.y=object.regions[i].bounding_box.y();
        region_msg.bounding_box.z=object.regions[i].bounding_box.z();
        Eigen::Affine3f temp(object.pose.matrix());
        tf::poseEigenToMsg((Eigen::Affine3d) temp, region_msg.pose.pose); // We consider the pose to be relatively to world
        region_msg.probability=object.regions[i].likelihood;
        msg.regions[i]=region_msg;
    }

    return msg;
}

void RosObjectDetails::fillObjectDetailsMarkers()
{
    for(unsigned int i=0; i<object->regions.size(); ++i)
        //if(object->has_handle)
    {
        visualization_msgs::Marker marker;
        marker.ns = "object_parts";
        //marker.id = x+(y*x_divisions)+(z*x_divisions*y_divisions);
        marker.id= i+object_frames.size()*object->regions.size();

        marker.header.frame_id = object_frames.back() ;
        marker.header.stamp = ros::Time::now();

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Rotate object part to world frame
        Eigen::Quaternion<float> rotation(object->regions[i].pose.rotation());
        marker.pose.orientation.x=rotation.x();
        marker.pose.orientation.y=rotation.y();
        marker.pose.orientation.z=rotation.z();
        marker.pose.orientation.w=rotation.w();

        Eigen::Translation3f translation(object->regions[i].pose.translation());
        marker.pose.position.x=translation.x();
        marker.pose.position.y=translation.y();
        marker.pose.position.z=translation.z();

        marker.scale.x=2*object->regions[i].bounding_box.x();
        marker.scale.y=2*object->regions[i].bounding_box.y();
        marker.scale.z=2*object->regions[i].bounding_box.z();

        marker.lifetime=ros::Duration(15);

        marker.color.a=0.6;
        marker.color.r=0;
        marker.color.g=i;
        marker.color.b=0;

        marker_array.markers.push_back(marker);
    }

    num_markers_published=object->regions.size();
}

void RosObjectDetails::clearMarkers()
{
    std::cout << "delete markers:" << std::endl;
    std::cout << "marker array size:" << marker_array.markers.size() << std::endl;
    for (int id=0; id < (int) num_markers_published; ++id)
    {
        visualization_msgs::Marker marker;
        //marker.id = x+(y*x_divisions)+(z*x_divisions*y_divisions);
        marker.id= id;
        marker.ns = "object_parts";
        //marker.header.frame_id = object_frames.back;
        marker.header.stamp = ros::Time::now();

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);

    }
}

void RosObjectDetails::broadcastTable()
{
    if(table_detected)
    {
        table_transform_stamped=tf::StampedTransform(table_transform, ros::Time::now(), table_parent_frame_id, table_frame_id);
        br.sendTransform(table_transform_stamped);
    }
}

void RosObjectDetails::broadcastObject()
{
    if(object_detected)
    {
        for(uint32_t t=0; t< object_transforms.size(); ++t)
        {
            object_transform_stamped=tf::StampedTransform(object_transforms[t], ros::Time::now(), object_parent_frame_id, object_frames[t]);
            br.sendTransform(object_transform_stamped);

            //handle_transform_stamped=tf::StampedTransform(handle_transform, ros::Time::now(), "object_1", "handle_frame");
            //br.sendTransform(handle_transform_stamped);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ist_object_details");
    ros::NodeHandle n;

    RosObjectDetails objRec(n);
    ros::Rate r(30);

    while(ros::ok())
    {
        objRec.broadcastTable();
        objRec.broadcastObject();
        //objRec.broadcastObjectCoordinateFrame();
        ros::spinOnce();
        r.sleep();
    }
}
