
#include "objectrecognition/models.h"
#include "objectrecognition_sv/object_model.h"

#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sstream>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> objectModel::viewportsVis(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	const std::string points="points";
	const std::string normals="normals";
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);

 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud, 0, 255, 0);

  	viewer->addPointCloud<pcl::PointNormal> (cloud, single_color, points,0);
  	viewer->addPointCloudNormals<pcl::PointNormal>(cloud,1,0.01,normals, 0);

  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, points);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, normals);
  	viewer->addCoordinateSystem (0.1);
  	viewer->initCameraParameters ();
  	return (viewer);
}
int main(int argc, char** argv)
{
double distanceStep= strtod(argv[2], NULL);


		typedef pcl::PointCloud<pcl::PointXYZ> pointCloudPointXYZ;
		typedef pointCloudPointXYZ::Ptr pointCloudPointXYZPtr;
		typedef pcl::PointCloud<pcl::Normal> pointCloudNormal;
		typedef pointCloudNormal::Ptr pointCloudNormalPtr;
		typedef pcl::PointCloud<pcl::PointNormal> pointCloudPointNormal;
		typedef pointCloudPointNormal::Ptr pointCloudPointNormalPtr;
     std::string filename = argv[1];
	// Create models object
	models<objectModelSV> modelsLibrary;

     std::string frame_id= "object_frame";
	pointCloudPointNormalPtr point_cloud_ptr=modelsLibrary.loadModel(filename, frame_id, 1.0);


	std::cout << "  Downsample dense surflet cloud... " << std::endl;
	std::cout << "   Surflet cloud size before downsampling: " << point_cloud_ptr->size() << std::endl;
 	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (point_cloud_ptr);
  	sor.setLeafSize (distanceStep,distanceStep,distanceStep);
  	sor.filter (*point_cloud_ptr);
	std::cout << "   Surflet cloud size after downsampling: " << point_cloud_ptr->size() << std::endl;
	std::cout << "  Done" << std::endl;

  	pcl::io::savePCDFileASCII (filename+"_uniform", *point_cloud_ptr);
		

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = objectModel::viewportsVis(point_cloud_ptr);

  	while (!viewer->wasStopped ())
  	{
   		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}
	
	//pcl::io::saveOBJFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision=5)
}
