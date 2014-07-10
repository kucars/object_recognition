#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
class RegionGrowingSegmentationRos
{

private:

    // PCL attributes
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud <pcl::Normal>::Ptr normals;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & point_cloud_msg)
    {
        ROS_INFO("ENTREI");
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_msg,*point_cloud_pcl);
        ROS_INFO("SAI");
        segment(point_cloud_pcl);
        return;
    }


    // ROS attributes
    ros::NodeHandle n_;
    ros::Subscriber point_cloud_sub_;

public:


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr segment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (cloud);
        //reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
        std::cout << "These are the indices of the points of the initial" << std::endl << "cloud that belong to the first cluster:" << std::endl;
        int counter = 0;
        while (counter < 5 || counter > clusters[0].indices.size ())
        {
            std::cout << clusters[0].indices[counter] << std::endl;
            counter++;
        }

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud(colored_cloud);
        while (!viewer.wasStopped ())
        {
        }

        return colored_cloud;
    }

    RegionGrowingSegmentationRos(ros::NodeHandle & n) : n_(n)
    {
        tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
        point_cloud_sub_ = n_.subscribe("/camera/depth_registered/points", 10, &RegionGrowingSegmentationRos::pointCloudCallback, this);
    }

};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "region_growing_segmentation");
    ros::NodeHandle n;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    /*if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }*/

    RegionGrowingSegmentationRos region_growing_segmentation(n);
    //region_growing_segmentation.segment(cloud);
    ros::spin();

    return (0);
}
