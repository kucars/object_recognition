#ifndef FILTERING
#define FILTERING

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

class filtering
{
	public:
	// Methods
		//static pcl::PointCloud<pcl::PointXYZ>::Ptr preFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float downsamplingStep);
		//static pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		//static pcl::PointCloud<pcl::PointNormal>::Ptr smoothingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		static pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,float downsamplingStep);
		static pcl::PointCloud<pcl::PointNormal>::Ptr downsampleXYZNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, float downsamplingStep);

};

#endif //#ifndef FILTERING

