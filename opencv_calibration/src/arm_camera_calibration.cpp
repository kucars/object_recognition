#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>



using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud ("vp1_target");
    p->removePointCloud ("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO ("Press q to begin the registration.\n");
    p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud ("source");
    p->removePointCloud ("target");


    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    if (!tgt_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
    if (!src_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");


    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
    std::string extension (".pcd");
    // Suppose the first argument is the actual test model
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string (argv[i]);
        // Needs to be at least 5: .plot
        if (fname.size () <= extension.size ())
            continue;

        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

        //check that the argument is a pcd file
        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile (argv[i], *m.cloud);
            //remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

            models.push_back (m);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }


    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);



    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO ("Press q to continue the registration.\n");
    p->spin ();

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}


float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "marker_detect");
    ros::NodeHandle n;

    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("arm");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.

    group.setPoseReferenceFrame("base_link");
    std::cout << group.getPlanningFrame() << std::endl;

    group.setEndEffectorLink("end_effector");


    std::cout << "get current state" << std::endl;
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    std::cout << "done" << std::endl;

    double min_x,min_y,min_z;
    double max_x,max_y,max_z;

    min_x=0.2;
    min_y=0.2;
    min_z=0.2;

    max_x=1.0;
    max_y=1.0;
    max_z=1.0;

    group.setWorkspace(min_x,min_y,min_z,max_x,max_y,max_z);



    /* moveit_msgs::Constraints constraints;
    moveit_msgs::PositionConstraint end_effector_constraint;
    end_effector_constraint.link_name="end_effector";
    end_effector_constraint.header.frame_id="base_link";
    end_effector_constraint.weight=1.0;
    shape_msgs::SolidPrimitive solid_primitive;
    solid_primitive.type=shape_msgs::SolidPrimitive::BOX;
    solid_primitive.dimensions.resize(3);
    solid_primitive.dimensions[0]=1.0;
    solid_primitive.dimensions[1]=1.0;
    solid_primitive.dimensions[2]=1.0;
    end_effector_constraint.constraint_region.primitives.push_back(solid_primitive);
    geometry_msgs::Pose pose;
    pose.orientation.w=1;
    pose.position.x=0.5;
    pose.position.y=0.5;
    pose.position.z=0.5;
    end_effector_constraint.constraint_region.primitive_poses.push_back(pose);
    constraints.position_constraints.push_back(end_effector_constraint);
    group.setPathConstraints(constraints);*/

    group.setGoalTolerance(0.03);

    moveit::planning_interface::MoveGroup::Plan my_plan;

    // specify that our target will be a random one
    geometry_msgs::PoseStamped random_pose;
    random_pose.header.frame_id="base_link";

    std::cout << "get end effector link:"<< group.getEndEffectorLink()<<std::endl;
    bool success;
    PointCloud::Ptr arm_cloud(new PointCloud);
    PointCloud::Ptr camera_cloud(new PointCloud);

    double range=0.2;
    double range_angle=0.2;

    int number_of_points=6;

    while(ros::ok() &&
          camera_cloud->points.size()!=number_of_points &&
          arm_cloud->points.size()!=number_of_points)
    {
        do
        {
            tf::Quaternion quat_tf=tf::createQuaternionFromRPY(-1.56,RandomFloat(-range,range),RandomFloat(-range,range));
            geometry_msgs::Quaternion quat_msg;
            tf::quaternionTFToMsg(quat_tf,quat_msg);
            //random_pose=group.getRandomPose();
            random_pose.pose.orientation=quat_msg;
            random_pose.pose.position.x=0.5+RandomFloat(-range,range);
            random_pose.pose.position.y=0.6+RandomFloat(-range,range);
            random_pose.pose.position.z=0.25+RandomFloat(-range,range);

            group.setPoseTarget(random_pose);
            success = group.plan(my_plan);
            //std::cout << random_pose.pose.position << std::endl;
        }
        while(!success && ros::ok());
        std::cout << "success" << std::endl;

        if(group.execute(my_plan))
            std::cout << "completed"<<std::endl;
        else
            std::cout << "didn't complete"<<std::endl;

        sleep(2.0);

        //std::cout << group.getPoseTarget() << std::endl;

        //std::cout << "current pose:" << group.getCurrentPose() << std::endl;


        std::string marker_link="ar_marker_4";
        std::string end_effector_link="end_effector";
        std::string camera_link="camera_link";

        std::string base_link="base_link";

        tf::TransformListener listener;
        // Get some point correspondences
        try
        {
            /////////////////////////////////////
            // Point in the end effector frame //
            /////////////////////////////////////

            tf::StampedTransform marker_end_effector_tf;
            listener.waitForTransform(base_link, end_effector_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(base_link, end_effector_link, ros::Time(0), marker_end_effector_tf);

            ///////////////////////////
            // Point in camera frame //
            ///////////////////////////

            tf::StampedTransform marker_camera_tf;
            listener.waitForTransform(camera_link, marker_link, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(camera_link, marker_link, ros::Time(0), marker_camera_tf);

            std::cout << "CHEGUEi aqui" << std::endl;

            PointT arm_point;
            arm_point.x=marker_end_effector_tf.getOrigin().x();
            arm_point.y=marker_end_effector_tf.getOrigin().y();
            arm_point.z=marker_end_effector_tf.getOrigin().z();
            arm_cloud->points.push_back(arm_point);

            std::cout << "arm_point:" << arm_point  << std::endl;

            PointT camera_point;
            camera_point.x=marker_camera_tf.getOrigin().x();
            camera_point.y=marker_camera_tf.getOrigin().y();
            camera_point.z=marker_camera_tf.getOrigin().z();
            camera_cloud->points.push_back(camera_point);

            std::cout << "camera_point:" << camera_point  << std::endl;

        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
    }

    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    for (size_t i = 1; i < arm_cloud->points.size (); ++i)
    {
        source = arm_cloud;
        target = camera_cloud;

        // Add visualization data
        //showCloudsLeft(source,  target);

        PointCloud::Ptr temp (new PointCloud);
        //cqaPCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        pairAlign (source, target, temp, pairTransform, true);

        //transform current pair into the global transform
        //pcl::transformPointCloud (*temp, *result, GlobalTransform);

        //update the global transform
        //GlobalTransform = pairTransform * GlobalTransform;

        //save aligned pair, transformed into the first cloud's frame
        //std::stringstream ss;
        //ss << i << ".pcd";
        //pcl::io::savePCDFile (ss.str (), *result, true);

    }
}

