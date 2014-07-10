#include "objectrecognition/filtering.h"

/*pcl::PointCloud<pcl::PointXYZ>::Ptr filtering::preFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float downsamplingStep)
{
	// Downsample point cloud using the resolution of the lowest resolution sensor
	if(downsamplingStep!=0)
		cloud=downsampleXYZ(cloud,downsamplingStep);
	// Apply a smoothing algorithm
	cloud=smoothing(cloud);
	return cloud;
}*/

/*pcl::PointCloud<pcl::PointXYZ>::Ptr filtering::smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	// Create a KD-Tree
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	// Output has the same type as the input one, it will be only smoothed
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ> ());

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

	// Optionally, a pointer to a cloud can be provided, to be set by MLS
	//pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	//mls.setOutputNormals (mls_normals);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (false);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.02);

	// Reconstruct
	mls.reconstruct(*mls_points);

	// Concatenate fields for saving
	//pcl::PointCloud<pcl::PointNormal> mls_cloud;
	// pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

	// Save output
	return mls_points;
}*/


/*pcl::PointCloud<pcl::PointNormal>::Ptr filtering::smoothingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	// Create a KD-Tree
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	// Output has the same type as the input one, it will be only smoothed
	pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ> ());

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

	// Optionally, a pointer to a cloud can be provided, to be set by MLS
	pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	mls.setOutputNormals (mls_normals);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (false);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.reconstruct(*mls_points);

	// Concatenate fields for saving
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::concatenateFields (*mls_points, *mls_normals, *mls_cloud);

	// Save output
	return mls_cloud;
}*/



pcl::PointCloud<pcl::PointXYZ>::Ptr filtering::downsampleXYZ(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, float downsamplingStep)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ> ());

  	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (downsamplingStep,downsamplingStep,downsamplingStep);
  	sor.filter (*cloudOut);

	return cloudOut;
}

pcl::PointCloud<pcl::PointNormal>::Ptr filtering::downsampleXYZNormal(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, float downsamplingStep)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudOut(new pcl::PointCloud<pcl::PointNormal> ());

  	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (downsamplingStep,downsamplingStep,downsamplingStep);
  	sor.filter (*cloudOut);

	return cloudOut;
}
