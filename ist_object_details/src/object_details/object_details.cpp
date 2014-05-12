#include "object_details/object_details.h"

double ObjectDetails::object_type_threshold;
double ObjectDetails::D;
double ObjectDetails::top_height_threshold;
double ObjectDetails::downsampling_step;
double ObjectDetails::inliers_circle_threshold;
double ObjectDetails::handle_distance_threshold;
double ObjectDetails::circle_ransac_distance_threshold;
double ObjectDetails::circle_minimum_radius;
double ObjectDetails::circle_maximum_radius;
double ObjectDetails::centroid_distance_threshold;
double ObjectDetails::vertical_height_threshold;
double ObjectDetails::hull_alpha;
double ObjectDetails::min_handle_points_percentage;
unsigned int ObjectDetails::neighbours;
bool ObjectDetails::remove_outliers;


const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::top    = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>( 0, 0, 1);
const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::bottom = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>( 0, 0,-1);
const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::left   = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>( 0, 1, 0);
const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::right  = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>( 0,-1, 0);
const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::front  = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>( 1, 0, 0);
const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> ObjectDetails::back   = Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign>(-1, 0, 0);

const std::vector<Eigen::Vector3f> ObjectDetails::discrete_orientations=boost::assign::list_of
(Eigen::Vector3f(0,0,1))				// vertical
(Eigen::Vector3f(1,0,0))				// front
(Eigen::Vector3f(sqrt(2)/2, -sqrt(2)/2, 0))		// front right
(Eigen::Vector3f(0,1,0))				// right
(Eigen::Vector3f(sqrt(2)/2, sqrt(2)/2, 0));		// back right
const double EPSILON = 0.01;
inline bool equalFloat(double a, double b)
{
    return fabs(a - b) < EPSILON;
}




ObjectDetails::PointCloudInPtr ObjectDetails::downsamplePointCloud(PointCloudInPtr _input_cloud, const double _downsampling_step)
{
	PointCloudInPtr filtered_aux_cloud(new PointCloudIn);

	pcl::VoxelGrid<pcl::PointXYZINormal> grid_filter;
	grid_filter.setInputCloud (_input_cloud);
	grid_filter.setLeafSize (_downsampling_step,_downsampling_step,_downsampling_step);
	grid_filter.filter (*filtered_aux_cloud);
	return filtered_aux_cloud;
}



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

void ObjectDetails::reestimateNormals(PointCloudInPtr ref_cloud)
{

	PointCloudInPtr cloud_aux(new PointCloudIn());
	*cloud_aux=*ref_cloud;

	//pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
	
	// Create the normal estimation class, and pass the input dataset to it
	// pcl::NormalEstimationOMP<PointXYZ, pcl::Normal> ne; ///VERSAO MAIS RAPIDA POIS USA multiplas threads! explorar isto
	pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> ne;
	ne.setInputCloud (cloud_aux);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//KdTreePtr (new KdTree);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZINormal> ());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	//ne.setSearchSurface (cloud);
	ne.setViewPoint(0,0,0);
	//std::cout << centroid << std::endl;
	// Output datasets
	/*if(radiusSearch)
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

	pcl::concatenateFields(*cloud_aux, *cloud_normals, *cloudWithNormals);

	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr nearest_neigh (new pcl::search::KdTree<pcl::PointXYZINormal> ());
	nearest_neigh->setInputCloud (ref_cloud);


	for(PointCloudIn::iterator p=cloudWithNormals->begin(); p<cloudWithNormals->end(); ++p)
	{
		std::vector< int > k_indices;
		std::vector< float > k_sqr_distances;
		nearest_neigh->nearestKSearch (*p, 1, k_indices, k_sqr_distances);
		if( (p->normal_x*ref_cloud->points[k_indices[0]].normal_x + p->normal_y*ref_cloud->points[k_indices[0]].normal_y + p->normal_z*ref_cloud->points[k_indices[0]].normal_z) < 0)
		{
			p->normal_x=-p->normal_x;
			p->normal_y=-p->normal_y;
			p->normal_z=-p->normal_z;
		}
	}
	
	ref_cloud=cloudWithNormals;

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(ref_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
}


ObjectDetails::PointCloudInPtr ObjectDetails::computeNormals(PointCloudInPtr cloud)
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



// Reflect on X
ObjectDetails::PointCloudInPtr ObjectDetails::zAxisReflect(float z_max, PointCloudInPtr point_cloud)
{		
	PointCloudInPtr reflectedCloud(new PointCloudIn);
	*reflectedCloud=*point_cloud;

	// subtract Z_max/2 (x_sym,y_sym) offset
	float z_subtract=z_max/2;
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->z-=z_subtract;
	}

	// reflect on the plane Z=0
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->z=-p->z;
		// Also reflect normal
		//p->normal_x=-p->normal_x;
		//p->normal_y=-p->normal_y;
		p->normal_z=-p->normal_z;

	}

	// add Z_max/2 offset
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->z+=z_subtract;
	}

	return reflectedCloud;
}

// Reflect on XY
ObjectDetails::PointCloudInPtr ObjectDetails::xyAxisReflect(float x_sym, float y_sym, PointCloudInPtr sceneClusterCloud)
{
	PointCloudInPtr reflectedCloud(new PointCloudIn);
	*reflectedCloud=*sceneClusterCloud;

	// subtract XY centroid (x_sym,y_sym) plane
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
			p->x-=x_sym;
			p->y-=y_sym;
	}

	// reflect on planes X=0 and Y=0
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x=-p->x;
		p->y=-p->y;

		// Also reflect normal
		p->normal_x=-p->normal_x;
		p->normal_y=-p->normal_y;
	}



	// add XY centroid offset
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x+=x_sym;
		p->y+=y_sym;
	}

	return reflectedCloud;
}

// Reflect on XYZ
ObjectDetails::PointCloudInPtr ObjectDetails::xyzAxisReflect(float x_sym, float y_sym, float z_max, PointCloudInPtr sceneClusterCloud)
{
	PointCloudInPtr reflectedCloud(new PointCloudIn);
	*reflectedCloud=*sceneClusterCloud;

	// subtract XYZ centroid (x_sym,y_sym,z_max/2)
	float z_subtract=z_max/2;
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x-=x_sym;
		p->y-=y_sym;
		p->z-=z_subtract;

	}

	// reflect on planes X=0, Y=0 and Z=0
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x=-p->x;
		p->y=-p->y;
		p->z=-p->z;

		// Also reflect normal
		p->normal_x=-p->normal_x;
		p->normal_y=-p->normal_y;
		p->normal_z=-p->normal_z;
	}

	// add XYZ centroid offset
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x+=x_sym;
		p->y+=y_sym;
		p->z+=z_subtract;
	}

	return reflectedCloud;
}

ObjectDetails::PointCloudInPtr ObjectDetails::planarXYReflect(PointCloudInPtr sceneClusterCloud)
{
	PointCloudInPtr reflectedCloud(new PointCloudIn);
	*reflectedCloud=*sceneClusterCloud;


	// reflect on planes X and Y after applying rotation and translation
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->x=-p->x;
		p->y=-p->y;

		// Also reflect normal
		p->normal_x=-p->normal_x;
		p->normal_y=-p->normal_y;
		p->normal_z=-p->normal_z;
	}

	return reflectedCloud;
}

ObjectDetails::PointCloudInPtr ObjectDetails::planarZReflect(PointCloudInPtr sceneClusterCloud)
{
	PointCloudInPtr reflectedCloud(new PointCloudIn);
	*reflectedCloud=*sceneClusterCloud;


	// reflect on planes X after applying rotation and translation
	for(PointCloudIn::iterator p=reflectedCloud->begin(); p<reflectedCloud->end(); ++p)
	{
		p->z=-p->z;

		// Also reflect normal
		//p->normal_x=-p->normal_x;
		//p->normal_y=-p->normal_y;
		p->normal_z=-p->normal_z;
	}

	return reflectedCloud;
}

void ObjectDetails::completeConsideringLineSymmetry(const Eigen::Affine3f & pose_temp)
{
	PointCloudInPtr aux_point_cloud(new PointCloudIn);
	aux_point_cloud->header=core_point_cloud->header;

	pcl::transformPointCloudWithNormals(*core_point_cloud, *aux_point_cloud, (Eigen::Affine3f) pose_temp.inverse());
	PointCloudInPtr planarXYReflectCloud = planarXYReflect(aux_point_cloud);
	pcl::transformPointCloudWithNormals(*planarXYReflectCloud, *aux_point_cloud, (Eigen::Affine3f) pose_temp);
	(*complete_point_cloud)+=(*aux_point_cloud);
	// Downsample using a grid filter
	complete_point_cloud=downsamplePointCloud(complete_point_cloud, downsampling_step);

	// Point cloud used for object details...
	(*point_cloud_complete_xyz_reflect)+=(*complete_point_cloud);

	// Reflect also on Z
	pcl::transformPointCloudWithNormals(*complete_point_cloud, *aux_point_cloud, (Eigen::Affine3f) pose_temp.inverse());
	PointCloudInPtr planarZReflectCloud = planarZReflect(aux_point_cloud);
	pcl::transformPointCloudWithNormals(*planarZReflectCloud, *aux_point_cloud, (Eigen::Affine3f) pose_temp);
	(*point_cloud_complete_xyz_reflect)+=(*aux_point_cloud);

	// Downsample using a grid filter
	point_cloud_complete_xyz_reflect=downsamplePointCloud(point_cloud_complete_xyz_reflect, downsampling_step);
}


void ObjectDetails::completeConsideringRotationalSymmetry(const double & x_sym, const double & y_sym, const double & z_max)
{
	// Reflect on XY
	PointCloudInPtr xyReflect=xyAxisReflect(x_sym, y_sym, core_point_cloud);
	xyReflect->header=complete_point_cloud->header;

	(*complete_point_cloud)+=(*xyReflect);

	// Downsample using a grid filter
	complete_point_cloud=downsamplePointCloud(complete_point_cloud, downsampling_step);
	(*point_cloud_complete_xyz_reflect)=(*complete_point_cloud);


	// Point cloud used for object details...

	// Reflect only on Z
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(complete_point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

  PointCloudInPtr zReflect=zAxisReflect(z_max, complete_point_cloud);
  zReflect->header=complete_point_cloud->header;
	(*point_cloud_complete_xyz_reflect)+=(*zReflect);

	// Reflect on XYZ
 	PointCloudInPtr xyzReflect=xyzAxisReflect(x_sym, y_sym, z_max, complete_point_cloud);
  xyzReflect->header=complete_point_cloud->header;
	(*point_cloud_complete_xyz_reflect)+=(*xyzReflect);

	// Downsample using a grid filter
	point_cloud_complete_xyz_reflect=downsamplePointCloud(point_cloud_complete_xyz_reflect, 0.01);
  /*viewer2 = viewportsVis(xyzReflect);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
  /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(point_cloud_complete_xyz_reflect);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

}



void ObjectDetails::computeDiscreteOrientation()
{
	double threshold =0.9;
	Eigen::Vector3f vertical_core_axis=eigen_vector[2];

	if(object_shape_type==SHAPE_TYPE_2_B) // Exception
	{
		vertical_core_axis=eigen_vector[0];
	}

	discrete_vertical_orientation=0;
	if(fabs(vertical_core_axis.dot(Eigen::Vector3f::UnitZ ()))>threshold)
	{
		// Vertical
		std::cout << " vertical" << std::endl;
		discrete_vertical_orientation=1;
	}
	else
	{
		// Horizontal
		std::cout << " horizontal" << std::endl;
		discrete_vertical_orientation=2;
	}
}















// This method assumes planar or rotational symmetry
void ObjectDetails::completeObjectPointCloudAlternative(PointCloudInPtr point_cloud)
{
	core_point_cloud->clear();
	core_point_cloud->header=point_cloud->header;

	handle_point_cloud->clear();
	handle_point_cloud->header=point_cloud->header;

	cluster_top->clear();
	cluster_top->header=point_cloud->header;

	cluster_top2D->clear();
	cluster_top2D->header=point_cloud->header;

	core2D->clear();
	core_top2D->header=point_cloud->header;

	complete_point_cloud->clear();
	complete_point_cloud->header=point_cloud->header;

	point_cloud_complete_xyz_reflect->clear();
	point_cloud_complete_xyz_reflect->header=point_cloud->header;
	//point_cloud=computeNormals(point_cloud);

	/////////////////////
	// Remove outliers //
	/////////////////////

	if(remove_outliers==true)
	{
		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *point_cloud << std::endl;

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
		sor.setInputCloud (point_cloud);
		sor.setMeanK (20);
		sor.setStddevMulThresh (1.0);
		sor.filter (*point_cloud);
	}
		
	////////////
	// Smooth //
	////////////
	
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	// Create a KD-Tree
	/*pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_ (new pcl::search::KdTree<pcl::PointXYZINormal>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	PointCloudInPtr mls_points (new PointCloudIn);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZINormal, pcl::PointXYZINormal> mls;
	 
	//mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (point_cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree_);
	mls.setSearchRadius (0.02);
	//mls.setMeanK(10);

  	// Reconstruct
  	mls.process(*mls_points);*/
	//point_cloud=mls_points;

	/*viewportsVis(point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	////////////////////////////////////////////////////
	// Downsample point cloud part using a voxel grid //
	////////////////////////////////////////////////////

	point_cloud=downsamplePointCloud(point_cloud, downsampling_step);

	// Project cluster top part on the XY plane
	core2D=planeProjection(point_cloud);

	// Downsample top part projected on the XY plane, using a grid filter
	core2D=downsamplePointCloud(core2D, downsampling_step);
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(core2D);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
	////////////////////////////////////////////
	// Compute PCA for the cluster projection //
	////////////////////////////////////////////

	double x_2D_centroid=0.0;
	double y_2D_centroid=0.0;
	Eigen::Transform<float, 3, Eigen::Affine> cluster_pose_temp;
	Eigen::Vector3f cluster_average_position;
	pcl::VectorAverage<float,3> cluster_vaverage;
	Eigen::Matrix<float, 3, 1> cluster_evalues;
	std::vector<Eigen::Matrix<float, 3, 1> > cluster_evector;
	Eigen::Vector3f cluster_bbox;
	cluster_evector.resize(3);
	Eigen::Vector4f cluster_min_pt, cluster_max_pt;
	computeBoundingBox(core2D, cluster_average_position, cluster_evalues, cluster_evector, cluster_bbox, cluster_pose_temp, cluster_min_pt, cluster_max_pt);
	x_2D_centroid=cluster_average_position[0];
	y_2D_centroid=cluster_average_position[1];

	std::cout << "CLUSTER AVERAGE POSITION:" << cluster_average_position << std::endl;

	///////////////////////
	// Get highest point //
	///////////////////////

	double z_max=0.00;

	for(size_t p=0; p<point_cloud->size(); ++p)
	{
		if(point_cloud->points[p].z > z_max)
		{
			z_max=point_cloud->points[p].z;
		}
	}


	double x_density_2D_centroid=0.0;
	double y_density_2D_centroid=0.0;



	for(PointCloudIn::iterator p=core2D->begin(); p<core2D->end(); ++p)
	{
		x_density_2D_centroid+=p->x;
		y_density_2D_centroid+=p->y;
	}

	x_density_2D_centroid/=core2D->size();
	y_density_2D_centroid/=core2D->size();

	///////////////////////
	// Compare centroids //
	///////////////////////

	double dx=x_2D_centroid-x_density_2D_centroid;
	double dy=y_2D_centroid-y_density_2D_centroid;
	double centroid_distance=sqrt(dx*dx+dy*dy);

	// Get the top part of the object
	pcl::PassThrough<pcl::PointXYZINormal> pass;
	pass.setInputCloud (point_cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z_max - top_height_threshold*z_max, z_max + top_height_threshold*z_max);
	pass.filter (*cluster_top);

	// Project cluster top part on the XY plane
	cluster_top2D=planeProjection(cluster_top);

	// Downsample top part projected on the XY plane, using a grid filter
	cluster_top2D=downsamplePointCloud(cluster_top2D, downsampling_step);



	///////////////////////////////////
	// Find handle (devide in parts) //
	///////////////////////////////////


	if(findHandle(point_cloud,core2D,centroid_distance))
	{
		std::cout << " Has handle." << std::endl;

		(*complete_point_cloud)+=(*core_point_cloud);
		std::cout << " Has handle then has rotational symmetry." << std::endl;
		completeConsideringRotationalSymmetry(circle_coeffs_refined[0], circle_coeffs_refined[1], z_max);
		(*complete_point_cloud)+=(*handle_point_cloud);
		(*point_cloud_complete_xyz_reflect)+=(*handle_point_cloud);
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(complete_point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	/*viewer2 = viewportsVis(point_cloud_complete_xyz_reflect);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
		return;
	}

	(*complete_point_cloud)+=(*core_point_cloud);


 	/* boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(point_cloud_complete_xyz_reflect);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/



 

	////////////////////////////////////////
	// Compute PCA for the top projection //
	////////////////////////////////////////

	double x_top_2D_centroid=0.0;
	double y_top_2D_centroid=0.0;
	Eigen::Transform<float, 3, Eigen::Affine> top_pose_temp;
	Eigen::Vector3f top_average_position;
	pcl::VectorAverage<float,3> top_vaverage;
	Eigen::Matrix<float, 3, 1> top_evalues;
	std::vector<Eigen::Matrix<float, 3, 1> > top_evector;
	Eigen::Vector3f top_bbox;
	top_evector.resize(3);
	Eigen::Vector4f top_min_pt, top_max_pt;
	computeBoundingBox(cluster_top2D, top_average_position, top_evalues, top_evector, top_bbox, top_pose_temp, top_min_pt, top_max_pt);
	x_top_2D_centroid=top_average_position[0];
	y_top_2D_centroid=top_average_position[1];



	bool shape_type_2b=false;
	if( (fabs(z_max-cluster_bbox[1])/          cluster_bbox[1] > object_type_threshold) &&  
			(fabs(z_max-cluster_bbox[2])/          cluster_bbox[2] > object_type_threshold) &&  
			(fabs(cluster_bbox[2]-cluster_bbox[1])/cluster_bbox[1] < object_type_threshold))
		shape_type_2b=true;
	//std::cout << "OBJECT SHAPE TYPE 2b: " << shape_type_2b << std::endl;
	//std::cout << cluster_bbox[1] << " " << cluster_bbox[2] << " " << z_max << std::endl;

	//std::cout << z_max << " " << cluster_max_pt.z() << std::endl;
	if( z_max>cluster_max_pt.z() || (z_max<cluster_max_pt.y() && shape_type_2b) ) // object aligned vertically
	{
		std::cout << "OBJECT ALIGNED VERTICALLY" << std::endl;

		top_average_position[0]=x_top_2D_centroid;
		top_average_position[1]=y_top_2D_centroid;
		top_average_position[2]=z_max/2.0;
		top_pose_temp=Eigen::Translation3f(top_average_position)*top_pose_temp.rotation().Identity();
		completeConsideringLineSymmetry(top_pose_temp);
	}
	else // object aligned horizontally
	{
		std::cout << "OBJECT ALIGNED HORIZONTALLY" << std::endl;

		cluster_average_position[0]=x_2D_centroid;
		cluster_average_position[1]=y_2D_centroid;
		cluster_average_position[2]=z_max/2.0;
		cluster_pose_temp=Eigen::Translation3f(cluster_average_position)*cluster_pose_temp.rotation();
		completeConsideringLineSymmetry(cluster_pose_temp);
	}


	reestimateNormals(complete_point_cloud);
}







void ObjectDetails::computeBoundingBox(PointCloudInPtr _point_cloud, Eigen::Vector3f & _position, Eigen::Vector3f & _eigen_values, std::vector<Eigen::Vector3f> & _eigen_vector, Eigen::Vector3f & _bounding_box, Eigen::Transform<float, 3, Eigen::Affine> & _pose, Eigen::Vector4f & min_pt, Eigen::Vector4f & max_pt)
{
	/////////////////
	// Compute PCA //
	/////////////////

	for(unsigned int i=0; i<_point_cloud->points.size(); ++i)
	{
		va.add(Eigen::Matrix<float, 3, 1> (_point_cloud->points[i].x, _point_cloud->points[i].y, _point_cloud->points[i].z));
	}

	va.doPCA(_eigen_values, _eigen_vector[0], _eigen_vector[1], _eigen_vector[2]);
	_position = va.getMean();
	va.reset();

	_eigen_values[0]=sqrt(_eigen_values[0]);
	_eigen_values[1]=sqrt(_eigen_values[1]);
	_eigen_values[2]=sqrt(_eigen_values[2]);

	Eigen::Matrix3f rotation_matrix;
	rotation_matrix << _eigen_vector[0], _eigen_vector[1], _eigen_vector[2];
	Eigen::AngleAxisf angleAxis;
	angleAxis = Eigen::Matrix3f(3,3);
	angleAxis = rotation_matrix;

	_pose=Eigen::Translation3f(_position)*angleAxis;
	centered_point_cloud=PointCloudInPtr (new PointCloudIn);
	pcl::transformPointCloudWithNormals(*_point_cloud, *centered_point_cloud, (Eigen::Affine3f) _pose.inverse());

	////////////////////////
	// Recompute position //
	////////////////////////

	pcl::getMinMax3D(*centered_point_cloud, min_pt, max_pt);

	Eigen::Vector3f position_temp;
	position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=( max_pt.z() + min_pt.z() )/2.00000;
	//std::cout << "position temp before:" << position_temp << std::endl;
	_position=_position+( _pose.rotation() * position_temp );
	_pose=Eigen::Translation3f(_position)*angleAxis;
	centered_point_cloud->clear();
	pcl::transformPointCloudWithNormals(*_point_cloud, *centered_point_cloud, (Eigen::Affine3f) _pose.inverse());

	//////////////////////////
	// Compute bounding box //
	//////////////////////////

	pcl::getMinMax3D(*centered_point_cloud, min_pt, max_pt);
	/*position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=( max_pt.z() + min_pt.z() )/2.00000;*/

	//std::cout << "position temp after:" << position_temp << std::endl;
	_bounding_box[X_o]=( max_pt.x() - min_pt.x() )/2.00000;
	_bounding_box[Y_o]=( max_pt.y() - min_pt.y() )/2.00000;
	_bounding_box[Z_o]=( max_pt.z() - min_pt.z() )/2.00000;

	_eigen_values[0]=_bounding_box[X_o];
	_eigen_values[1]=_bounding_box[Y_o];
	_eigen_values[2]=_bounding_box[Z_o];
 	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(centered_point_cloud);
		while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
	//std::cout << "_pose:" << _pose.matrix() << std::endl;
}










void ObjectDetails::computeBodyBoundingBox(PointCloudInPtr _point_cloud, Eigen::Vector3f & _position, Eigen::Vector3f & _eigen_values, std::vector<Eigen::Vector3f> & _eigen_vector, Eigen::Vector3f & _bounding_box, Eigen::Transform<float, 3, Eigen::Affine> & _pose, Eigen::Vector4f & min_pt, Eigen::Vector4f & max_pt)
{
	/////////////////
	// Compute PCA //
	/////////////////

	// Project on the XY plane

	ObjectDetails::PointCloudInPtr xy_point_cloud=planeProjection(_point_cloud);

	// Downsample using a grid filter
	xy_point_cloud=downsamplePointCloud(xy_point_cloud, downsampling_step);

	for(unsigned int i=0; i<xy_point_cloud->points.size(); ++i)
	{
		va.add(Eigen::Matrix<float, 3, 1> (xy_point_cloud->points[i].x, xy_point_cloud->points[i].y, xy_point_cloud->points[i].z));
	}

	va.doPCA(_eigen_values, _eigen_vector[0], _eigen_vector[1], _eigen_vector[2]);

std::cout << "EIGEN VALUUUUUES: " << _eigen_values<< std::endl;

	///////////////////////
	// Get highest point //
	///////////////////////

	double z_max=0.00;

	for(size_t p=0; p<_point_cloud->size(); ++p)
	{
		if(_point_cloud->points[p].z > z_max)
		{
			z_max=_point_cloud->points[p].z;
		}
	}
	double temp_eigen_value=z_max*0.5;
	int index=0;
	if(temp_eigen_value>_eigen_values[1]&&temp_eigen_value<=_eigen_values[2])
	{
		Eigen::Vector3f aux_vec=_eigen_vector[1];
		_eigen_vector[1]=Eigen::Vector3f::UnitZ();
		_eigen_vector[0]=aux_vec;
		double aux_value=_eigen_values[1];
		_eigen_values[1]=temp_eigen_value;
		_eigen_values[0]=aux_value;
		std::cout << "ola" << std::endl;
	}
	else if(temp_eigen_value>_eigen_values[2])
	{
			_eigen_vector[0]=_eigen_vector[1];
			_eigen_values[0]=_eigen_values[1];
			_eigen_vector[1]=_eigen_vector[2];
			_eigen_values[1]=_eigen_values[2];
			_eigen_vector[2]=Eigen::Vector3f::UnitZ();
			_eigen_values[2]=temp_eigen_value;
		std::cout << "ole" << std::endl;
	}
	else
	{
			_eigen_vector[0]=Eigen::Vector3f::UnitZ();
			_eigen_values[0]=temp_eigen_value;
		std::cout << "oli" << std::endl;
	}


	_position = va.getMean();

	_position[2]=temp_eigen_value;

	va.reset();

	_eigen_values[0]=sqrt(_eigen_values[0]);
	_eigen_values[1]=sqrt(_eigen_values[1]);
	_eigen_values[2]=sqrt(_eigen_values[2]);

	Eigen::Matrix3f rotation_matrix;
	rotation_matrix << _eigen_vector[0], _eigen_vector[1], _eigen_vector[2];
	if(rotation_matrix.determinant()<0)
		_eigen_vector[0]=-_eigen_vector[0];
	rotation_matrix << _eigen_vector[0], _eigen_vector[1], _eigen_vector[2];

	Eigen::AngleAxisf angleAxis;
	angleAxis = Eigen::Matrix3f(3,3);
	angleAxis = rotation_matrix;

	_pose=Eigen::Translation3f(_position)*angleAxis;
	centered_point_cloud=PointCloudInPtr (new PointCloudIn);
	pcl::transformPointCloudWithNormals(*_point_cloud, *centered_point_cloud, (Eigen::Affine3f) _pose.inverse());

	////////////////////////
	// Recompute position //
	////////////////////////

	pcl::getMinMax3D(*centered_point_cloud, min_pt, max_pt);

	Eigen::Vector3f position_temp;
	position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=( max_pt.z() + min_pt.z() )/2.00000;
	//std::cout << "position temp before:" << position_temp << std::endl;
	_position=_position+( _pose.rotation() * position_temp );
	_pose=Eigen::Translation3f(_position)*angleAxis;
	centered_point_cloud->clear();
	pcl::transformPointCloudWithNormals(*_point_cloud, *centered_point_cloud, (Eigen::Affine3f) _pose.inverse());

	//////////////////////////
	// Compute bounding box //
	//////////////////////////

	pcl::getMinMax3D(*centered_point_cloud, min_pt, max_pt);
	/*position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=( max_pt.z() + min_pt.z() )/2.00000;*/

	//std::cout << "position temp after:" << position_temp << std::endl;
	_bounding_box[X_o]=( max_pt.x() - min_pt.x() )/2.00000;
	_bounding_box[Y_o]=( max_pt.y() - min_pt.y() )/2.00000;
	_bounding_box[Z_o]=( max_pt.z() - min_pt.z() )/2.00000;

	_eigen_values[0]=_bounding_box[X_o];
	_eigen_values[1]=_bounding_box[Y_o];
	_eigen_values[2]=_bounding_box[Z_o];
 	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(centered_point_cloud);
		while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
	//std::cout << "_pose:" << _pose.matrix() << std::endl;
}





void ObjectDetails::applyBoundingBoxOrientationRules()
{
	// PIOR CODIGO DE SEMPRE -> MELHORAR ISTO
	double biggest_component=0.0;
	int biggest_component_index=0;
	for(int i=0; i<3 ;++i)
	{
		if(bounding_box[i]>biggest_component)
		{
			biggest_component=bounding_box[i];
			biggest_component_index=i;
		}
	}

	double second_biggest_component=0.0;
	int second_biggest_component_index=0;
	for(int i=0; i<3 ;++i)
	{
		if(i!=biggest_component_index)
			if(bounding_box[i]>second_biggest_component)
			{
				second_biggest_component=bounding_box[i];
				second_biggest_component_index=i;
			}
	}

	int third_biggest_component_index=0;
	for(int i=0; i<3 ;++i)
	{
		if(i!=biggest_component_index&&i!=second_biggest_component_index)
			third_biggest_component_index=i;
	}

	eigen_values[2]=bounding_box[biggest_component_index];
	eigen_values[1]=bounding_box[second_biggest_component_index];
	eigen_values[0]=bounding_box[third_biggest_component_index];
	bounding_box=eigen_values;

	std::vector<Eigen::Vector3f> aux_eigen_vector;
	aux_eigen_vector=eigen_vector;
	eigen_vector[2]=aux_eigen_vector[biggest_component_index];
	eigen_vector[1]=aux_eigen_vector[second_biggest_component_index];
	eigen_vector[0]=aux_eigen_vector[third_biggest_component_index];

	//std::cout << "eigen_values:" << eigen_values << std::endl;
}

void ObjectDetails::getBodyHandleIndices(PointCloudInPtr point_cloud)
{
	has_handle=false;

	handle_indices->indices.clear();
	body_indices->indices.clear();

	core_point_cloud->header=point_cloud->header;
	core_point_cloud->points.clear();
	handle_point_cloud->header=point_cloud->header;
	handle_point_cloud->points.clear();

	for(unsigned int i=0; i < point_cloud->size(); ++i)
	{
		if(point_cloud->points[i].intensity>0.5)
		{
			handle_indices->indices.push_back(i);
		}
		else
		{
			body_indices->indices.push_back(i);
		}
	}

	if((double)handle_indices->indices.size()>0)
		has_handle=true;
 	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(point_cloud);
		while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	// Fill core point cloud
	for(unsigned int core_index=0; core_index < body_indices->indices.size(); ++core_index)
	{
		core_point_cloud->push_back(point_cloud->points[body_indices->indices[core_index]]);
		core_point_cloud->back().intensity=0;
	}
 /*	viewer2 = viewportsVis(core_point_cloud);
		while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
	// Fill handle point cloud
	for(unsigned int handle_index=0; handle_index < handle_indices->indices.size(); ++handle_index)
	{

		handle_point_cloud->push_back(point_cloud->points[handle_indices->indices[handle_index]]);
		handle_point_cloud->back().intensity=4;
	}

	point_cloud->points.clear();
}

void ObjectDetails::computeObjectDetails(PointCloudInPtr point_cloud_object_details, PointCloudInPtr point_cloud)
{

	getBodyHandleIndices(point_cloud_object_details);

	Eigen::Vector4f min_pt,max_pt;
	computeBodyBoundingBox(core_point_cloud, position, eigen_values, eigen_vector, bounding_box, pose, min_pt, max_pt);
	applyBoundingBoxOrientationRules();
	computeObjectShapeType();
	computeDiscreteOrientation(); // See if object is in vertical or horizontal orientation (depends on shape)
	computeObjectPose(); // Compute axis direction


	computeObjectPartsNumber();
	computeObjectSymmetryType();
	computeBodyRegions(point_cloud);

	if(has_handle)
	{
		computeBoundingBox(handle_point_cloud, handle_position, handle_eigen_values, handle_eigen_vector, handle_bounding_box, handle_pose, min_pt, max_pt);
		computeHandlePose();
		computeHandleRegions(point_cloud);
	}

	//printObjectType();

	//point_cloud=core_point_cloud;

	//std::cout << "pose:" << pose.matrix() << std::endl;
	complete_point_cloud->clear();
	centered_point_cloud->clear();
	(*complete_point_cloud)+=(*core_point_cloud);
	(*complete_point_cloud)+=(*handle_point_cloud);



	



	pcl::transformPointCloudWithNormals(*complete_point_cloud, *centered_point_cloud, (Eigen::Affine3f) pose.inverse());
	
	/*for(PointCloudIn::iterator p=centered_point_cloud->begin(); p < centered_point_cloud->end(); ++p)
	{	

		std::cout << p->intensity << " ";
	}

	std::cout << std::endl;*/

 	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(centered_point_cloud);

	while (!viewer2->wasStopped ())
	{
		viewer2->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}*/

}


void ObjectDetails::computeHandleRegions(PointCloudInPtr point_cloud)
{
	// handle region...
	unsigned int type_id_=1;
	double likelihood_=0.7;
	unsigned int part_id_=4;
	regions.push_back(ObjectRegion(part_id_, type_id_, handle_pose, handle_bounding_box, likelihood_));
}

void ObjectDetails::computeObjectShapeType()
{
	float d1=eigen_values[2];
	float d2=eigen_values[1];
	float d3=eigen_values[0];

	bool E21=false;
	bool E32=false;

//	std::cout << "EIGEN VALUES: " << eigen_values << std::endl;
//	std::cout << "type threshond: " << object_type_threshold << std::endl;
	// Test if bigger and second bigger components are equal
	if( fabs(d1-d2)/d1 < (object_type_threshold) )
		E21=true;

	// Test if smaller and second smaller components are equal
	if( fabs(d3-d2)/d2 < (object_type_threshold) )
		E32=true;

	// Select object type given the components
	if(E21&&E32)
		object_shape_type=SHAPE_TYPE_1;
	else if( (!E21) && (E32) )
		object_shape_type=SHAPE_TYPE_2_A;
	else if( (E21) && (!E32))
		object_shape_type=SHAPE_TYPE_2_B;
	else
		object_shape_type=SHAPE_TYPE_3;
}


void ObjectDetails::computeObjectShapeType(const Eigen::Vector3f & _eigen_values)
{
	float d1=_eigen_values[2];
	float d2=_eigen_values[1];
	float d3=_eigen_values[0];

	bool E21=false;
	bool E32=false;

//	std::cout << "EIGEN VALUES: " << eigen_values << std::endl;
//	std::cout << "type threshond: " << object_type_threshold << std::endl;
	// Test if bigger and second bigger components are equal
	if( fabs(d1-d2)/d1 < (object_type_threshold) )
		E21=true;

	// Test if smaller and second smaller components are equal
	if( fabs(d3-d2)/d2 < (object_type_threshold) )
		E32=true;

	// Select object type given the components
	if(E21&&E32)
		object_shape_type=SHAPE_TYPE_1;
	else if( (!E21) && (E32) )
		object_shape_type=SHAPE_TYPE_2_A;
	else if( (E21) && (!E32))
		object_shape_type=SHAPE_TYPE_2_B;
	else
		object_shape_type=SHAPE_TYPE_3;
}


void ObjectDetails::computeHandlePose()
{
	Eigen::Matrix3f rotation_matrix;

	unsigned int z_index=0;
	float biggest=0.0;

	//////////////////////////////////
	// Get vertical direction index //
	//////////////////////////////////

	for(unsigned int i=0; i < 3 ; ++i)
	{
		float temp = fabs(handle_eigen_vector[i].dot(Eigen::Vector3f::UnitZ()));
		if(temp>biggest)
		{
			biggest=temp;
			z_index=i;
		}
	}
	if(handle_eigen_vector[z_index].dot(Eigen::Vector3f::UnitZ())<0)
	{
		handle_eigen_vector[z_index]=-handle_eigen_vector[z_index];
	}

	//////////////////////////////
	// Get body direction index //
	//////////////////////////////

	biggest=0.0;
	unsigned int body_direction_index=0;
	Eigen::Vector3f temp_body_direction=handle_position-position;

	for(unsigned int i=0; i < 3 ; ++i)
	{
		if(i==z_index)
		{
			continue;
		}

		float temp = fabs(handle_eigen_vector[i].dot(temp_body_direction));
		if(temp>biggest)
		{
			biggest=temp;
			body_direction_index=i;
		}
	}

	if(handle_eigen_vector[body_direction_index].dot(temp_body_direction)>0)
	{
		handle_eigen_vector[body_direction_index]=-handle_eigen_vector[body_direction_index];
	}

	unsigned int x_index=0;

	for(unsigned int i=0; i < 3 ; ++i)
	{
		if(i!=z_index&&i!=body_direction_index)
		{
			x_index=i;
			break;
		}
	}

	handle_eigen_values[2]=handle_bounding_box[z_index];
	handle_eigen_values[1]=handle_bounding_box[body_direction_index];
	handle_eigen_values[0]=handle_bounding_box[x_index];
	handle_bounding_box=handle_eigen_values;
	std::vector<Eigen::Vector3f> aux_handle_eigen_vector;
	aux_handle_eigen_vector=handle_eigen_vector;
	handle_eigen_vector[2]=aux_handle_eigen_vector[z_index];
	handle_eigen_vector[1]=aux_handle_eigen_vector[body_direction_index];
	handle_eigen_vector[0]=aux_handle_eigen_vector[body_direction_index].cross(aux_handle_eigen_vector[z_index]);
	
	rotation_matrix << handle_eigen_vector[0], handle_eigen_vector[1], handle_eigen_vector[2];

	if(rotation_matrix.determinant()<0)
		handle_eigen_vector[0]=-handle_eigen_vector[0];
	rotation_matrix << handle_eigen_vector[0], handle_eigen_vector[1], handle_eigen_vector[2];

	Eigen::AngleAxisf angleAxis;
	angleAxis = Eigen::Matrix3f(3,3);
	angleAxis = rotation_matrix;

	handle_pose=Eigen::Translation3f(handle_position)*angleAxis;

	handle_pose_world=handle_pose;

	handle_pose=pose.inverse()*handle_pose; // handle pose in object frame
}


void ObjectDetails::computeObjectPose()
{
	Eigen::Matrix3f rotation_matrix;

	/////////////////////////////////////////////////////////////////////////////////////
	// Disambiguate principal components directions using the reasoning in HANDLE wiki //
	/////////////////////////////////////////////////////////////////////////////////////

	// check biggest component predominant direction
	size_t biggest_component_direction=0;
	for(size_t i=1; i<3; ++i)
	{
		if(fabs(eigen_vector[2][i])>fabs(eigen_vector[2][i-1]))
		{
			biggest_component_direction=i;
		}
	}

	// check second biggest component predominant direction
	size_t second_biggest_component_direction=0;
	for(size_t i=1; i<3; ++i)
	{
		if(fabs(eigen_vector[1][i])>fabs(eigen_vector[1][i-1]))
		{
			second_biggest_component_direction=i;
		}
	}

	// check smallest component predominant direction
	size_t smallest_component_direction=0;
	for(size_t i=1; i<3; ++i)
	{
		if(fabs(eigen_vector[0][i])>fabs(eigen_vector[0][i-1]))
		{
			smallest_component_direction=i;
		}
	}

	if(object_shape_type==SHAPE_TYPE_1) // sphere-like
	{
		rotation_matrix << front, left, top;
	}
	else if(object_shape_type==SHAPE_TYPE_2_A && discrete_vertical_orientation==1) // tall cylinder-like (vertical)
	{
//		std::cout << "ENTROU AQUIIIIIIIIIIIIIIIIIIII shape type 2A" << std::endl;

		// If biggest component predominant direction is Z, ensure it points up (Z_w top)
		if(biggest_component_direction==Z_w)
		{
			rotation_matrix << front, left, top;
		}
		else if(biggest_component_direction==Y_w)
		{
			rotation_matrix << back, bottom, right;
		}
		else if(biggest_component_direction==X_w)
		{
			rotation_matrix << left, bottom, back;
		}
		//exit(0);
	}
	else if(object_shape_type==SHAPE_TYPE_2_B && discrete_vertical_orientation==1) // fat cylinder-like
	{
//		std::cout << "shape type 2B" << std::endl;

		// If smallest component predominant direction is Z, ensure it points up (Z_w top)
		if(smallest_component_direction==Z_w)
		{
			rotation_matrix << front, left, top;
		}
		else if(smallest_component_direction==Y_w)
		{
			rotation_matrix << back, bottom, right;
		}
		else if(smallest_component_direction==X_w)
		{
			rotation_matrix << left, bottom, back;
			//ROS_INFO("EXAAAAAACTO: %f",rotation_matrix.determinant());
		}
	}
	else if ((object_shape_type==SHAPE_TYPE_2_B && discrete_vertical_orientation==2))
	{
//		std::cout << "NTROU AQUIIII" << std::endl;
		// If biggest component predominant direction is Z, ensure it points up (Z_w top)

		// If smallest component predominant direction is Y, ensure it points right (Y_w right)
		if(smallest_component_direction==Y_w)
		{
			//std::cout << "biggest component along predominant direction is Y" << eigen_vector[2].transpose() << std::endl;
			if(eigen_vector[0][Y_w]>0)
			{
				eigen_vector[0]=-eigen_vector[0];
			}
//			std::cout << "E IPSLOOONN" << std::endl;
		}
		// If smallest component predominant direction is X, ensure it points back (Z_w back)
		else if(smallest_component_direction==X_w)
		{
			//std::cout << "biggest component along predominant direction is X" << eigen_vector[2].transpose() << std::endl;
			if(eigen_vector[0][X_w]>0)
			{
				eigen_vector[0]=-eigen_vector[0];
			}
//			std::cout << "E XIIIIIS" << std::endl;
		}
		else
		{
			std::cout << "E ZEEEE FODA-SE" << std::endl;
			exit(-1);
		}

		rotation_matrix << eigen_vector[2], eigen_vector[1], eigen_vector[0];

		if(rotation_matrix.determinant()<0)
			eigen_vector[2]=-eigen_vector[2];
		rotation_matrix << eigen_vector[2], eigen_vector[1], eigen_vector[0];
	}
	else if(object_shape_type==SHAPE_TYPE_3 || (object_shape_type==SHAPE_TYPE_2_A && discrete_vertical_orientation==2)) // parallelepiped-like
	{
		// If biggest component predominant direction is Z, ensure it points up (Z_w top)
		if(biggest_component_direction==Z_w)
		{
			//std::cout << "biggest component along predominant direction is Z:" << eigen_vector[2].transpose() << std::endl;
			if(eigen_vector[2][Z_w]<0)
			{
				eigen_vector[2]=-eigen_vector[2];
			}
			// If second biggest component predominant direction is Y, ensure it points to the left (Y_w left)
			if(second_biggest_component_direction==Y_w)
			{
				if(eigen_vector[1][Y_w]<0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}
			// If second biggest component predominant direction is X, ensure it points to the front (Y_w front)
			else
			{
				if(eigen_vector[1][X_w]<0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}
		}
		// If biggest component predominant direction is Y, ensure it points right (Y_w right)
		else if(biggest_component_direction==Y_w)
		{
			//std::cout << "biggest component along predominant direction is Y" << eigen_vector[2].transpose() << std::endl;
			if(eigen_vector[2][Y_w]>0)
			{
				eigen_vector[2]=-eigen_vector[2];
			}

			// If second biggest component predominant direction is Z, ensure it points to the bottom (Y_w bottom)
			if(second_biggest_component_direction==Z_w)
			{
				if(eigen_vector[1][Z_w]>0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}

			// If second biggest component predominant direction is X, ensure it points to the front (Y_w front)
			else
			{
				if(eigen_vector[1][X_w]<0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}
		}
		// If biggest component predominant direction is X, ensure it points back (Z_w back)
		else
		{
			//std::cout << "biggest component along predominant direction is X" << eigen_vector[2].transpose() << std::endl;
			if(eigen_vector[2][X_w]>0)
			{
				eigen_vector[2]=-eigen_vector[2];
			}

			// If second biggest component predominant direction is Y, ensure it points to the right (Y_w right)
			if(second_biggest_component_direction==1)
			{
				if(eigen_vector[1][Y_w]>0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}
			// If second biggest component predominant direction is Z, ensure it points to the bottom (Y_w bottom)
			else
			{
				if(eigen_vector[1][Z_w]>0)
				{
					eigen_vector[1]=-eigen_vector[1];
				}
			}
		}

		rotation_matrix << eigen_vector[0], eigen_vector[1], eigen_vector[2];

		if(rotation_matrix.determinant()<0)
			eigen_vector[0]=-eigen_vector[0];
		rotation_matrix << eigen_vector[0], eigen_vector[1], eigen_vector[2];
	}


	Eigen::AngleAxisf angleAxis;
	angleAxis = Eigen::Matrix3f(3,3);
	angleAxis = rotation_matrix;

	//pcl::transformPointCloudWithNormals(*core_point_cloud, *core_point_cloud, (Eigen::Affine3f) pose.inverse());
	pose=Eigen::Translation3f(position)*angleAxis;
	//pcl::transformPointCloudWithNormals(*core_point_cloud, *core_point_cloud, (Eigen::Affine3f) pose);

	//centered_point_cloud=PointCloudInPtr (new PointCloudIn);
	//pcl::transformPointCloudWithNormals(*core_point_cloud, *centered_point_cloud, (Eigen::Affine3f) pose.inverse());
}

void ObjectDetails::computeObjectSymmetryType()
{
	if(object_shape_type==SHAPE_TYPE_1)
	{
		object_symmetry_type=SYMMETRY_CUBE_TYPE; // Cube type
	}
	else if(object_shape_type==SHAPE_TYPE_2_A)
	{
		object_symmetry_type=SYMMETRY_CHALK_STICK_TYPE_Z; // Chalk type
	}
	else if(object_shape_type==SHAPE_TYPE_2_B)
	{
		object_symmetry_type=SYMMETRY_CHALK_STICK_TYPE_Z; // Chalk type
	}
	else if(object_shape_type==SHAPE_TYPE_3)
	{
		object_symmetry_type=SYMMETRY_BRICK_TYPE; // Brick type
	}
	else
		object_symmetry_type=ANY_SYMMETRY_TYPE;
}

void ObjectDetails::computeObjectPartsNumber()
{
	float dx=bounding_box[X_o];
	float dy=bounding_box[Y_o];
	float dz=bounding_box[Z_o];

	if((dx < D) && (dy < D))
	{
		if((dy <= dz) && (dz < D))
		{
			// 1 part
			object_parts_number=1;

		}
		else if((D <= dz) && (dz < 2*D))
		{
			// 2 parts
			object_parts_number=2;
		}
		else if((2*D <= dz))
		{
			// 4 parts
			object_parts_number=3;
		}
	}
	else if((dx < D) && (dy >= D) && (dy < 2*D))
	{
		if((dy <= dz) && (dz < 2*D))
		{
			// 4 parts
			object_parts_number=4;
		}
		else if(dz >= 2*D)
		{
			// 6 parts
			object_parts_number=6;
		}
	}
	else if((dx >= D) && (dy >= D) && (dx < 2*D) && (dy < 2*D))
	{
		if((dz >= D) && (dz < 2*D))
		{
			// 8 parts
			object_parts_number=8;
		}
		else if((dz >= 2*D))
		{
			// 12 parts
			object_parts_number=12;
		}
	}
	else if((dx < D) && (dy >= 2*D) && (dz >= 2*D))
	{
		// 9 parts
		object_parts_number=9;
	}
	else if((dx >= D) && (dx <= 2*D) && (dy >= 2*D) && (dz >= 2*D))
	{
		// 18 parts
		object_parts_number=18;
	}
	else if((dx >= 2*D) && (dy >= 2*D) && (dz >= 2*D))
	{
		// 26 parts
		object_parts_number=26;
	}
	else
	{
		object_parts_number=-1;
	}
}

ObjectDetails::PointCloudInPtr ObjectDetails::planeProjection(PointCloudInConstPtr cloud)
{
	PointCloudInPtr cloud_projected (new PointCloudIn);

	// Create a set of planar coefficients (XY plane with z=0)  ax+by+cz+d=0 a=b=d=0 z=1
	 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	  coefficients->values.resize (4);
	  coefficients->values[0] = coefficients->values[1] = 0;
	  coefficients->values[2] = 1.0;
	  coefficients->values[3] = 0;

	  // Create the filtering object
//	  ROS_INFO("OLA");
	  pcl::ProjectInliers<pcl::PointXYZINormal> proj;
	  proj.setModelType (pcl::SACMODEL_PLANE);
	  proj.setInputCloud (cloud);
	  proj.setModelCoefficients (coefficients);
	  proj.filter (*cloud_projected);
//	  ROS_INFO("ADEUS");

	  return cloud_projected;
}

void ObjectDetails::printObjectType()
{
	std::cout << "Object shape type: ";
	if(object_shape_type==1)
		std::cout << "1 (All dimensions are similar. E.g. ball)" << std::endl;
	else if(object_shape_type==2)
		std::cout << "2A (Singular dimension is the largest. E.g. Tall cylinders)" << std::endl;
	else if(object_shape_type==3)
		std::cout << "2B (Singular dimension is the smallest. E.g. Fat cylinders)" << std::endl;
	else if(object_shape_type==4)
		std::cout << "3 (All dimensions are distinct. E.g. parallelepiped)" << std::endl;
}

void ObjectDetails::printDiscreteOrientation()
{
	std::cout << "Horizontal orientation ";
	if(discrete_horizontal_orientation==1)
		std::cout << "1 vertical (WRONG!!!!)" << std::endl;
	else if(discrete_horizontal_orientation==2)
		std::cout << "2 front" << std::endl;
	else if(discrete_horizontal_orientation==3)
		std::cout << "2 front right (Singular dimension is the smallest. E.g. Fat cylinders)" << std::endl;
	else if(discrete_horizontal_orientation==4)
		std::cout << "3 back right" << std::endl;

}

void ObjectDetails::setTopIsOpenedLikelihood(PointCloudInPtr & point_cloud_top_projected)
{
	for(unsigned int i=0; i< point_cloud_top_projected->size(); ++i)
	{
		va.add(Eigen::Matrix<float, 3, 1> (point_cloud_top_projected->points[i].x, point_cloud_top_projected->points[i].y, point_cloud_top_projected->points[i].z));
	}

	// Handle eigen values and principal components
	Eigen::Vector3f top_eigen_values, top_eigen_vector1, top_eigen_vector2, top_eigen_vector3;
	va.doPCA(top_eigen_values, top_eigen_vector1, top_eigen_vector2, top_eigen_vector3);
	Eigen::Vector3f top_position;
	top_position = va.getMean();
	va.reset();
	top_position[2]=0;
	top_eigen_vector1[0]=0.0;
	top_eigen_vector2[1]=0.0;
	top_eigen_vector3[2]=0.0;

	//////////////////////////////
	// Compute top bounding box //
	//////////////////////////////
	Eigen::Matrix3f rotation_matrix;
	Eigen::AngleAxisf angleAxis;

	rotation_matrix << top_eigen_vector1, top_eigen_vector2, top_eigen_vector3;
	if(rotation_matrix.determinant()<0)
		handle_eigen_vector[0]=-handle_eigen_vector[0];
	rotation_matrix << handle_eigen_vector[0], handle_eigen_vector[1], handle_eigen_vector[2];
	angleAxis = rotation_matrix;

	Eigen::Affine3f top_pose=Eigen::Translation3f(top_position)*angleAxis;
	PointCloudInPtr top_centered_point_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::transformPointCloudWithNormals(*point_cloud_top_projected, *top_centered_point_cloud, (Eigen::Affine3f) top_pose.inverse());

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*top_centered_point_cloud, min_pt, max_pt);
	Eigen::Vector2f top_bounding_box;

	top_bounding_box[X_o]=fabs(max_pt.x());
	top_bounding_box[Y_o]=fabs(max_pt.y());

	// Create the filtering object
	PointCloudInPtr cloud_filtered_outside(new PointCloudIn);
	pcl::PassThrough<pcl::PointXYZINormal> pass;

	pass.setInputCloud (top_centered_point_cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-top_bounding_box[X_o], top_bounding_box[X_o]);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_outside);

	pass.setInputCloud (cloud_filtered_outside);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-top_bounding_box[Y_o], top_bounding_box[Y_o]);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_outside);

	PointCloudInPtr cloud_filtered_inside(new pcl::PointCloud<pcl::PointXYZINormal>);

	pass.setInputCloud(top_centered_point_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-top_bounding_box[X_o]+0.9*top_bounding_box[X_o], top_bounding_box[X_o]+0.9*top_bounding_box[X_o]);
	pass.filter (*cloud_filtered_inside);

	pass.setInputCloud(cloud_filtered_inside);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-top_bounding_box[Y_o]+0.9*top_bounding_box[Y_o], top_bounding_box[Y_o]+0.9*top_bounding_box[Y_o]);
	pass.filter (*cloud_filtered_inside);

//	top_is_opened_likelihood=1.0-(double)cloud_filtered_inside->size()/cloud_filtered_outside->size();
	top_is_opened_likelihood=0.5;
//	std::cout << "TOP IS OPENED LIKELIHOOD: " << top_is_opened_likelihood << std::endl;
}

bool ObjectDetails::findHandle(PointCloudInPtr point_cloud, PointCloudInPtr point_cloud_projected, const double & centroid_distance)
{
	std::cout << "point cloud size: " << point_cloud->size() << std::endl;
	has_handle=false;

	for(PointCloudIn::iterator p=point_cloud->begin(); p < point_cloud->end(); ++p )
	{
		p->intensity=0;
	}
	
	/*if(centroid_distance<centroid_distance_threshold)
	{
		std::cout << "NO HANDLE -> CENTROID DISTANCE:" << centroid_distance << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}*/

	/////////////////
	// concave HULL //
	/////////////////

	PointCloudInPtr cloud_concave_hull (new PointCloudIn);

	pcl::ModelCoefficients::Ptr concave_hull_coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr concave_hull_inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZINormal> seg;

	// Optional
	seg.setOptimizeCoefficients (true);

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.003);

	seg.setInputCloud (point_cloud_projected);
	seg.segment (*concave_hull_inliers, *concave_hull_coefficients);

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZINormal> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (point_cloud_projected);
	proj.setModelCoefficients (concave_hull_coefficients);
	proj.filter (*cloud_concave_hull);
	
	// Create a concave Hull representation of the projected inliers
	PointCloudInPtr cloud_hull (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::ConcaveHull<pcl::PointXYZINormal> chull;
	chull.setAlpha(hull_alpha);
	//chull.setDimension(3);
	chull.setInputCloud (cloud_concave_hull);
	chull.reconstruct (*cloud_hull);
  	

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(point_cloud_projected);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/
	
	/*viewer2 = viewportsVis(cloud_hull);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/





	pcl::SampleConsensusModelCircle2D<pcl::PointXYZINormal>::Ptr circle_model (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZINormal> (cloud_hull));

	pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac (circle_model);
	ransac.setMaxIterations (10000);
	ransac.setDistanceThreshold (circle_ransac_distance_threshold);
	ransac.computeModel();

	Eigen::VectorXf circle_coeffs;
	ransac.getModelCoefficients (circle_coeffs);

	std::vector<int> inliers;
	ransac.getInliers (inliers);

	circle_model->optimizeModelCoefficients(inliers, circle_coeffs, circle_coeffs_refined);



	// DECIDE IF IT IS A CIRCLE OR NOT



	//std::cerr << "concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
	if((double)cloud_hull->points.size ()<15)
	{
		std::cout << "too few points:" << circle_coeffs_refined[2] << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}

	if((double)circle_coeffs_refined[2]<circle_minimum_radius)
	{
		std::cout << "Circle too small:" << circle_coeffs_refined[2] << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}

	if((double)circle_coeffs_refined[2]>circle_maximum_radius)
	{
		std::cout << "Circle too big:" << circle_coeffs_refined << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		std::cout << "CIRCLE MAXIMUM RADIUS: " << circle_maximum_radius << std::endl;
		return false;
	}

	double inliers_ratio = (double)inliers.size()/(double)cloud_hull->size();

	if(inliers_ratio<inliers_circle_threshold)
	{
		std::cout << "Circle not detected:" << inliers_ratio << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}


	std::cout << "Circle detected -> " << " inliers ratio: " <<inliers_ratio << " inliers circle threshold: " << inliers_circle_threshold << " circle radius: "<< circle_coeffs_refined[2] <<std::endl;
	// Label handle point
	for(PointCloudIn::iterator p=point_cloud->begin(); p < point_cloud->end(); )
	{
		double dist=sqrt((p->x-circle_coeffs_refined[0])*(p->x-circle_coeffs_refined[0])+(p->y-circle_coeffs_refined[1])*(p->y-circle_coeffs_refined[1]));

		if(dist > (circle_coeffs_refined[2]+handle_distance_threshold) )
		{
			handle_point_cloud->push_back(*p);
			handle_point_cloud->back().intensity=1;
		}
		else
		{
			core_point_cloud->push_back(*p);
		}
		p=point_cloud->erase(p);
	}
	std::cout << "handle cloud size: " << handle_point_cloud->size() << std::endl;
	std::cout << "core cloud size: " << core_point_cloud->size() << std::endl;
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(core_point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(handle_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}*/
	// Remove handle outliers
	pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
	sor.setInputCloud (handle_point_cloud);
	sor.setMeanK ( round(handle_point_cloud->size()*0.8) );
	sor.setStddevMulThresh (1.0);
	PointCloudInPtr temp_cloud (new PointCloudIn);
	sor.filter (*temp_cloud);
	handle_point_cloud=temp_cloud;
	/*viewer2 = viewportsVis(handle_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}*/


	/////////////////////////////
	// Compute handle centroid //
	/////////////////////////////

	double x_handle_centroid=0.0;
	double y_handle_centroid=0.0;
	for(PointCloudIn::iterator p=handle_point_cloud->begin(); p < handle_point_cloud->end(); ++p)
	{
		x_handle_centroid+=p->x;
		y_handle_centroid+=p->y;
	}

	x_handle_centroid/=handle_point_cloud->size();
	y_handle_centroid/=handle_point_cloud->size();

	double dist=sqrt((x_handle_centroid-circle_coeffs_refined[0])*(x_handle_centroid-circle_coeffs_refined[0])+(y_handle_centroid-circle_coeffs_refined[1])*(y_handle_centroid-circle_coeffs_refined[1]));

	std::cout << " dist:" << dist << std::endl;
	double percent=(double)handle_point_cloud->size() / (double)(handle_point_cloud->size()+core_point_cloud->size());

	std::cout << "PERCENT: " << percent << " param: "<< min_handle_points_percentage<< std::endl;
	if(percent>min_handle_points_percentage && (dist>(double)circle_coeffs_refined[2]+handle_distance_threshold))
	{
		//std::cout << "HANDLE POINTS NUMBER:" << handle_point_cloud->size() << std::endl;
		has_handle=true;
	}
	else
	{
		std::cout << "HANDLE POINTS NUMBER:" << handle_point_cloud->size() << " dist:" << dist << std::endl;

		for(PointCloudIn::iterator p=handle_point_cloud->begin(); p < handle_point_cloud->end(); )
		{
			p->intensity=0;
			core_point_cloud->push_back(*p);
			p=handle_point_cloud->erase(p);
		}

		(*core_point_cloud)+=(*handle_point_cloud);
		handle_point_cloud->clear();
		has_handle=false;

	}


/*viewer2 = viewportsVis(handle_point_cloud);
	while (!viewer2->wasStopped ())
 	{
		viewer2->spinOnce (100);
 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 	}*/

	return has_handle;
}







bool ObjectDetails::findHandle(PointCloudInPtr point_cloud, PointCloudInPtr point_cloud_projected)
{
	has_handle=false;

	for(PointCloudIn::iterator p=point_cloud->begin(); p < point_cloud->end(); ++p )
	{
		p->intensity=0;
	}

	/////////////////
	// concave HULL //
	/////////////////

	PointCloudInPtr cloud_concave_hull (new PointCloudIn);

	pcl::ModelCoefficients::Ptr concave_hull_coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr concave_hull_inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZINormal> seg;

	// Optional
	seg.setOptimizeCoefficients (true);

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.003);

	seg.setInputCloud (point_cloud_projected);
	seg.segment (*concave_hull_inliers, *concave_hull_coefficients);

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZINormal> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (point_cloud_projected);
	proj.setModelCoefficients (concave_hull_coefficients);
	proj.filter (*cloud_concave_hull);

	// Create a concave Hull representation of the projected inliers
	PointCloudInPtr cloud_hull (new PointCloudIn);
	pcl::ConcaveHull<pcl::PointXYZINormal> chull;
	chull.setAlpha(hull_alpha);
	chull.setInputCloud (cloud_concave_hull);
	chull.reconstruct (*cloud_hull);

//	std::cerr << "concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;

	pcl::SampleConsensusModelCircle2D<pcl::PointXYZINormal>::Ptr circle_model (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZINormal> (cloud_hull));

	pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac (circle_model);
	ransac.setMaxIterations (10000);
	ransac.setDistanceThreshold (circle_ransac_distance_threshold);
	ransac.computeModel();

	Eigen::VectorXf circle_coeffs;
	ransac.getModelCoefficients (circle_coeffs);



	std::vector<int> inliers;
	ransac.getInliers (inliers);

	circle_model->optimizeModelCoefficients(inliers, circle_coeffs, circle_coeffs_refined);

	// DECIDE IF IT IS A CIRCLE OR NOT

	double inliers_ratio = (double)inliers.size()/(double)cloud_hull->size();

	if(inliers_ratio<inliers_circle_threshold)
	{
		std::cout << "Circle not detected:" << inliers_ratio << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}


	if((double)circle_coeffs_refined[2]<circle_minimum_radius)
	{
		std::cout << "Circle too small:" << circle_coeffs_refined << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}
	std::cout << "CIRCLE MAXIMUM RADIUS: " << circle_maximum_radius << std::endl;
	if((double)circle_coeffs_refined[2]>circle_maximum_radius)
	{
		std::cout << "Circle too big:" << circle_coeffs_refined << std::endl;
		(*core_point_cloud)=(*point_cloud);
		point_cloud->points.clear();
	 	has_handle=false;
		return false;
	}

	std::cout << "Circle detected -> " << " inliers ratio: " <<inliers_ratio << " inliers circle threshold: " << inliers_circle_threshold << " circle radius: "<< circle_coeffs_refined[2] <<std::endl;
	// Label handle point
	for(PointCloudIn::iterator p=point_cloud->begin(); p < point_cloud->end(); )
	{
		double dist=sqrt((p->x-circle_coeffs_refined[0])*(p->x-circle_coeffs_refined[0])+(p->y-circle_coeffs_refined[1])*(p->y-circle_coeffs_refined[1]));

		if(dist > (circle_coeffs_refined[2]+handle_distance_threshold) )
		{
			handle_point_cloud->push_back(*p);
			handle_point_cloud->back().intensity=1;
		}
		else
		{
			core_point_cloud->push_back(*p);
		}
		p=point_cloud->erase(p);
	}


	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(handle_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}*/
	// Remove handle outliers
	pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
	sor.setInputCloud (handle_point_cloud);
	sor.setMeanK ( (handle_point_cloud->size()-1) );
	sor.setStddevMulThresh (1.0);
	sor.filter (*handle_point_cloud);
	/*viewer2 = viewportsVis(handle_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}*/
	double percent=(double)handle_point_cloud->size() / (double)(handle_point_cloud->size()+core_point_cloud->size());
	if(percent>min_handle_points_percentage)
	{
		//std::cout << "HANDLE POINTS NUMBER:" << handle_point_cloud->size() << std::endl;
		has_handle=true;
	}
	else
	{
		//std::cout << "HANDLE POINTS NUMBER:" << handle_point_cloud->size() << std::endl;

		for(PointCloudIn::iterator p=handle_point_cloud->begin(); p < handle_point_cloud->end(); )
		{
			p->intensity=0;
			core_point_cloud->push_back(*p);
			p=handle_point_cloud->erase(p);
		}

		(*core_point_cloud)+=(*handle_point_cloud);
		handle_point_cloud->clear();
		has_handle=false;

	}

	return has_handle;
}





void ObjectDetails::computeBodyRegions(PointCloudInPtr point_cloud)
{
	PointCloudInPtr core_point_cloud_aux ( new PointCloudIn);
	is_tool=false;
	is_shape_type_2b_mug=false;
	double type_id_;
	double likelihood_;
	double part_id_;
	regions.clear();
	int x_divisions(1), y_divisions(1), z_divisions(3); // object is divided in three parts along z

	if(discrete_vertical_orientation==2)
	{
		Eigen::Vector3f vertical_axis;

		double biggest_vertical=0.0;
		int biggest_vertical_index=0;

		for(unsigned int i=0; i < eigen_vector.size() ; ++i)
		{
			double aux=fabs(eigen_vector[i].dot(Eigen::Vector3f::UnitZ ()));
			if(aux > biggest_vertical)
			{
				biggest_vertical=aux;
				biggest_vertical_index=i;
			}
		}

		std::cout << "VERTICAL THRESHOLD: " << vertical_height_threshold << std::endl;

		if(bounding_box[biggest_vertical_index]<vertical_height_threshold)
		{
				z_divisions=2;
				is_tool=true;
		}
	}


	if(has_handle && object_shape_type==SHAPE_TYPE_2_B)
	{
		std::cout << "OBJECT ESQUISITOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
		z_divisions=1;
		is_shape_type_2b_mug=true;
	}

	double z_dimension, y_dimension, x_dimension;

	double z_scale, y_scale, x_scale;

	double z_threshold, y_threshold, x_threshold;

	double dimension_x=bounding_box.x();
	double dimension_y=bounding_box.y();
	double dimension_z=bounding_box.z();
	if(object_shape_type==SHAPE_TYPE_2_B)
	{
		x_dimension=dimension_z;
    y_dimension=dimension_y;
		z_dimension=dimension_x;
	}
	else
	{
		x_dimension=dimension_x;
    y_dimension=dimension_y;
		z_dimension=dimension_z;
	}

	//if(object_shape_type==SHAPE_TYPE_2_B)
	

	x_scale=2*x_dimension/x_divisions;
	y_scale=2*y_dimension/y_divisions;
	z_scale=2*z_dimension/z_divisions;

	x_threshold=(x_dimension/x_divisions)*(x_divisions-1); 
	y_threshold=(y_dimension/y_divisions)*(y_divisions-1);
	z_threshold=(z_dimension/z_divisions)*(z_divisions-1);

	int total_ind=0;
	for(int x=0; x < x_divisions; ++x)
	{
		for(int y=0; y < y_divisions; ++y)
		{
			for(int z=0; z < z_divisions; ++z)
			{

				Eigen::Transform<float, 3, Eigen::Affine> region_pose;
				Eigen::Vector3f offset_before=Eigen::Vector3f((x*(x_scale) - x_threshold), (y*(y_scale) - y_threshold), (z*(z_scale) - z_threshold));
			
				Eigen::AngleAxisf angleAxis;

				//region_pose.translate(offset_before);
				region_pose=Eigen::Translation3f(offset_before)*angleAxis.Identity();
				//std::cout << "POSE OF THE OBJECT before rotationg:" << offset_before << " POSE OF THE OBJECT AFTER rotationg:"<< offset<<std::endl;

				
				Eigen::Vector3f region_bounding_box(x_dimension/x_divisions, y_dimension/y_divisions, z_dimension/z_divisions);

				//std::cout << "BOUNDING BOX !!!!: " << bounding_box << std::endl;

				if(z_divisions==1)
				{
						part_id_=5;
						likelihood_=0.7;
				}
				else if(z_divisions==2)
				{
						if(z==0)
						{
							// handle region...
							part_id_=4; //BOTTOM END id = 4(see ist_msgs/ObjectPart)
							likelihood_=0.5;
						}
						else if(z==1)
						{
							// usable region...
							part_id_=5;
							likelihood_=0.5;
						}
				}
				else
				{
					if(z==0)
					{
						// bottom region...
						part_id_=3; //BOTTOM END id = 4(see ist_msgs/ObjectPart)
						likelihood_=0.7;
					}
					else if(z==1)
					{
						// middle region...
						part_id_=2;
						likelihood_=1.0;
					}
					else if(z==2)
					{
						// top region...
						part_id_=1;
						likelihood_=0.7;
					}
					else
						continue;
				}
				type_id_=0;

				regions.push_back(ObjectRegion(part_id_, type_id_, region_pose, region_bounding_box,likelihood_));



				//Define your cube with two points in space:
     				Eigen::Vector4f maxPoint;
      				maxPoint[0]=region_bounding_box.x();  // define minimum point x
      				maxPoint[1]=region_bounding_box.y();  // define minimum point y
      				maxPoint[2]=region_bounding_box.z();  // define minimum point z
     				Eigen::Vector4f minPoint;
				minPoint[0]=-region_bounding_box.x();  // define max point x
				minPoint[1]=-region_bounding_box.y();  // define max point y
				minPoint[2]=-region_bounding_box.z();  // define max point z


					
				PointCloudIn cropped_point_cloud;

               			pcl::CropBox<pcl::PointXYZINormal> cropFilter;
        			cropFilter.setInputCloud (point_cloud);
				cropFilter.setTransform(region_pose.inverse()*pose.inverse());
               			cropFilter.setMin(minPoint);
               			cropFilter.setMax(maxPoint);
   				cropFilter.filter (cropped_point_cloud); 

				
				for(PointCloudIn::iterator p=cropped_point_cloud.begin(); p < cropped_point_cloud.end(); ++p)
				{

					p->intensity=part_id_;
					//std::cout << core_point_cloud->points[crop_indices[i]].intensity << std::endl;
					//teste->push_back(core_point_cloud->points[crop_indices[i]]);
				}
				(*core_point_cloud_aux)+=cropped_point_cloud;
			

				total_ind+=cropped_point_cloud.size();

				/*PointCloudInPtr aux_point_cloud(new PointCloudIn);
				aux_point_cloud->header=core_point_cloud->header;

				pcl::transformPointCloudWithNormals(*core_point_cloud_aux, *aux_point_cloud, (Eigen::Affine3f) region_pose.inverse()*pose.inverse());

 				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = viewportsVis(aux_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}
				
				pcl::transformPointCloudWithNormals(*teste, *aux_point_cloud, (Eigen::Affine3f) region_pose.inverse()*pose.inverse());
 				viewer2 = viewportsVis(aux_point_cloud);

				while (!viewer2->wasStopped ())
			 	{
					viewer2->spinOnce (100);
			 		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
			 	}*/

				std::cout << "total ind: "<< total_ind << std::endl;
			}
	
		}
	}

	core_point_cloud=core_point_cloud_aux;
	for(PointCloudIn::iterator p=core_point_cloud->begin(); p < core_point_cloud->end(); )
	{
		if(p->intensity==0)
			p=core_point_cloud->erase(p);
		else 
			++p;
	}

	std::cout << "core pcl size:" << point_cloud->size() << std::endl;
	std::cout << "CROP INDICES SIZE:" << total_ind<<std::endl;
}

