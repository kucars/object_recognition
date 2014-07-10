#include "objectrecognition/object_model.h"

bool objectModel::radiusSearch;
float objectModel::radius;
float objectModel::neighbours;
int objectModel::idNext=0;

unsigned int objectModel::distanceBins;

using namespace pcl;



void objectModel::computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloudCompareNormals)
{
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
	
	// Create the normal estimation class, and pass the input dataset to it
	// pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne; ///VERSAO MAIS RAPIDA POIS USA multiplas threads! explorar isto
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	//std::cout << centroid << std::endl;
	// Output datasets
	if(radiusSearch)
	{
	// Use all neighbors in a sphere of radius 'radius'
		ne.setRadiusSearch(radius);
	}
	else
	{
	// Use 'neighbours' neighbors
 		ne.setKSearch (neighbours);
	}  
	// Compute the features
	ne.compute(*cloud_normals);
	
	pcl::KdTreeFLANN<pcl::PointNormal>::Ptr nearestPointTree (new pcl::KdTreeFLANN<pcl::PointNormal> ());
	nearestPointTree->setInputCloud (cloudCompareNormals);
	std::vector< int > k_indices(1);
	std::vector< float > k_sqr_distances(1);
	// Concatenate point XYZ with it's normals
	pcl::concatenateFields(*cloud, *cloud_normals, *cloudWithNormals);
	// Normals point ouside!!!
	for(pcl::PointCloud<pcl::PointNormal>::iterator it=cloudWithNormals->begin(); it< cloudWithNormals->end(); it++)
	{
		nearestPointTree->nearestKSearch (*it, 1, k_indices,  k_sqr_distances);
		float dotProduct=it->normal_x*cloudCompareNormals->points[k_indices[0]].normal_x+
				 it->normal_y*cloudCompareNormals->points[k_indices[0]].normal_y+
				 it->normal_z*cloudCompareNormals->points[k_indices[0]].normal_z;

		if(dotProduct<0)
		{
			it->normal_x*=-1;
			it->normal_y*=-1;
			it->normal_z*=-1;
		}
	}
	/*boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer3 = objectModel::viewportsVis(cloudWithNormals);

  	while (!viewer3->wasStopped ())
  	{
   		viewer3->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/


	*cloudCompareNormals=*cloudWithNormals;
	//return cloudWithNormals;
}





// ISTO TA MAL
pcl::PointCloud<pcl::PointNormal>::Ptr objectModel::computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{


	//pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
	
	// Create the normal estimation class, and pass the input dataset to it
	// pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne; ///VERSAO MAIS RAPIDA POIS USA multiplas threads! explorar isto
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//KdTreePtr (new KdTree);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	//ne.setSearchSurface (cloud);
	pcl::compute3DCentroid(*cloud,centroid);
	ne.setViewPoint(centroid(0),centroid(1),centroid(2));
	//std::cout << centroid << std::endl;
	// Output datasets
	if(radiusSearch)
	{
	// Use all neighbors in a sphere of radius 'radius'
		ne.setRadiusSearch(radius);
	}
	else
	{
	// Use 'neighbours' neighbors
 		ne.setKSearch (neighbours);
	}  
	// Compute the features
	ne.compute(*cloud_normals);

	// Normals point ouside!!!
	for(pcl::PointCloud<pcl::Normal>::iterator it=cloud_normals->begin(); it< cloud_normals->end(); it++)
	{
		it->normal_x*=-1;
		it->normal_y*=-1;
		it->normal_z*=-1;
	}
	
	// Concatenate point XYZ with it's normals
	pcl::concatenateFields(*cloud, *cloud_normals, *cloudWithNormals);

	return cloudWithNormals;
}



pcl::PointCloud<pcl::PointNormal>::Ptr objectModel::computeSceneNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
	
	// Create the normal estimation class, and pass the input dataset to it
	// pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne; ///VERSAO MAIS RAPIDA POIS USA multiplas threads! explorar isto
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);


	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	//tree=pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	//ne.setSearchSurface (cloud);

	ne.setViewPoint(0,0,0);
	//std::cout << centroid << std::endl;
	// Output datasets
	if(radiusSearch)
	{
	// Use all neighbors in a sphere of radius 'radius'
		ne.setRadiusSearch(radius);
	}
	else
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
  	viewer->addPointCloudNormals<pcl::PointNormal>(cloud,1,1.0,normals, 0);

  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, points);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, normals);
  	viewer->addCoordinateSystem (0.1);
  	viewer->initCameraParameters ();
  	return (viewer);
}

// convert quaternion to euler
void objectModel::quaternionToEuler(const Eigen::Quaternion<float> & q, float & roll, float & pitch, float & yaw) 
{
	float test = q.x()*q.z() + q.y()*q.w();

	if (test > 0.49999) 
	{ // singularity at north pole
		
		roll = 0.0; // symmetric!!
		//roll = 2 * atan2(q.z(),q.w());
		pitch = PI/2.0;
		yaw = 0.0;
		return;
	}
	if (test < -0.49999) 
	{ // singularity at south pole

		roll = 0.0; // symmetric!!
		//roll = -2 * atan2(q.z(),q.w());
		pitch = - PI/2.0;
		yaw = 0.0;
		return;
	}
    float sqx = q.x()*q.x();
    float sqy = q.y()*q.y();
    //float sqz = q.z()*q.z();

    roll = atan2f(2*q.x()*q.w()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqy);
    pitch = asinf(2*test);
    	//yaw = atan2f(2*q.z()*q.w()-2*q.y()*q.x() , 1 - 2*sqz - 2*sqy);
	yaw=0.0; // symmetric!!

}
	
// convert euler angles to quaternion
void objectModel::eulerToQuaternion(Eigen::Quaternion<float> & q, const float & roll, const float & pitch, const float & yaw)
{
	float roll2=roll*0.5;
	float pitch2=pitch*0.5;
	float yaw2=yaw*0.5;
	// Assuming the angles are in radians.
	float c1 = cos(roll2);
	float s1 = sin(roll2);
    float c2 = cos(pitch2);
   	float s2 = sin(pitch2);
    float c3 = cos(yaw2);
    float s3 = sin(yaw2);
    float c1c2 = c1*c2;
    float s1s2 = s1*s2;
    q.w() =c1c2*c3 - s1s2*s3;
   	q.x() =s1*c2*c3 + c1*s2*s3;
   	q.y() =c1*s2*c3 - s1*c2*s3;
	q.z() =c1c2*s3 + s1s2*c3;
	q.normalize();



	return;
}



