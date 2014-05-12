#ifndef ELLIPSOID_DESCRIPTION
#define ELLIPSOID_DESCRIPTION

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PointIndices.h>

#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/crop_box.h>
 
//#include <pcl/filters/crop_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/vector_average.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <Eigen/Eigen>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include "defines.h"
#include "region.h"

using namespace cv;
using namespace std;

int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
/** @function thresh_callback */
void find_countours(Mat & src_gray, Mat & canny_output )
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// Detect edges using canny
	cv::Canny( src_gray, canny_output, thresh, thresh*2, 3 );
	// Find contours
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Draw contours
	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
	for(unsigned int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	 }

	// Show in a window
	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Contours", drawing );
	src_gray=drawing;
}

class ObjectDetails
{
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

	private:
		// Static definitions (object coordinate axis related to world)
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> top;
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> bottom;
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> left;
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> right;
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> front;
		static const Eigen::Matrix<float, 3 ,1 , Eigen::DontAlign> back;

		// Object type threshold
		static double object_type_threshold;

		// Top height threshold
		static double top_height_threshold;

		// Downsampling step
		static double downsampling_step;

		// Object parts threshold
		static double D;

		// Handle minimum number of points
		static double min_handle_points_percentage;

		static double circle_ransac_distance_threshold;

		static double inliers_circle_threshold;

		static double handle_distance_threshold;

		static double circle_minimum_radius;

		static double circle_maximum_radius;

		static double centroid_distance_threshold;

		static double vertical_height_threshold;

		// Pre processing
		static bool remove_outliers;

		static double hull_alpha;
		pcl::VectorAverage<float,3> va;
		pcl::VectorAverage<float,3> handle_va;

		// Eigen values and principal components
		Eigen::Vector3f eigen_values;

		std::vector<Eigen::Vector3f> eigen_vector;
		std::vector<Eigen::Vector3f> handle_eigen_vector;

		// Object position
		Eigen::Vector3f position;

		// Handle eigen values and principal components
		Eigen::Vector3f handle_eigen_values, handle_eigen_vector1, handle_eigen_vector2, handle_eigen_vector3;

		// Handle position
		Eigen::Vector3f handle_position;

		// Circle coefficients
        Eigen::VectorXf circle_coeffs_refined;

        pcl::PointIndices::Ptr handle_indices;
        pcl::PointIndices::Ptr body_indices;

        double top_is_opened_likelihood;

		/////////////
		// METHODS //
		/////////////

		////////////////////////////////////////////
		// Methods for object details computation //
		////////////////////////////////////////////

		// Compute object type
		void computeObjectShapeType();

		void computeObjectShapeType(const Eigen::Vector3f & _eigen_values);

		// Compute object symmetry type
		void computeObjectSymmetryType();

		// Compute object parts number
		void computeObjectPartsNumber();

		// Compute object regions
		void computeBodyRegions(PointCloudInPtr point_cloud);

		// Compute object pose
		void computeObjectPose();

		void computeHandlePose();

		void computeHandleRegions(PointCloudInPtr point_cloud);

		void computeDiscreteOrientation();

		void computeBoundingBox(PointCloudInPtr _point_cloud, Eigen::Vector3f & _position, Eigen::Vector3f & _eigen_values, std::vector<Eigen::Vector3f> & _eigen_vector, Eigen::Vector3f & _bounding_box, Eigen::Transform<float, 3, Eigen::Affine> & _pose, Eigen::Vector4f & min_pt, Eigen::Vector4f & max_pt);

		void computeBoundingBox(PointCloudInPtr _point_cloud, Eigen::Vector3f & _position, Eigen::Vector3f & _eigen_values, std::vector<Eigen::Vector3f> & _eigen_vector, Eigen::Vector3f & _bounding_box, Eigen::Transform<float, 3, Eigen::Affine> & _pose);

		void computeBodyBoundingBox(PointCloudInPtr _point_cloud, Eigen::Vector3f & _position, Eigen::Vector3f & _eigen_values, std::vector<Eigen::Vector3f> & _eigen_vector, Eigen::Vector3f & _bounding_box, Eigen::Transform<float, 3, Eigen::Affine> & _pose, Eigen::Vector4f & min_pt, Eigen::Vector4f & max_pt);


		//void computeBoundingBox(PointCloudInPtr point_cloud);

		void getBodyHandleIndices(PointCloudInPtr point_cloud);
		void computeZreflectedPointCloud(PointCloudInPtr point_cloud);

		////////////////////////////////////////
		// Methods for point cloud refinement //
		////////////////////////////////////////

		PointCloudInPtr downsamplePointCloud(PointCloudInPtr _input_cloud, const double _downsampling_step);

		//void completeConsideringPlanarSymmetry(const double & x_centroid, const double & y_centroid, const double & z_max);

		void completeConsideringRotationalSymmetry(const double & x_sym, const double & y_sym, const double & z_max);

		void completeConsideringLineSymmetry(const Eigen::Affine3f & pose_temp);

		PointCloudInPtr planeProjection(PointCloudInConstPtr cloud);

		// Reflect on Z
		PointCloudInPtr zAxisReflect(float z_max, PointCloudInPtr sceneClusterCloud);

		// Reflect on XY
		PointCloudInPtr xyAxisReflect(float x_sym, float y_sym, PointCloudInPtr sceneClusterCloud);

		// Reflect on XYZ
		PointCloudInPtr xyzAxisReflect(float x_sym, float y_sym, float z_max, PointCloudInPtr sceneClusterCloud);

		// Planar XY reflection
		PointCloudInPtr planarXYReflect(PointCloudInPtr sceneClusterCloud);

		// Planar Z reflection
		PointCloudInPtr planarZReflect(PointCloudInPtr sceneClusterCloud);


		// Find circle on object top
		bool findHandle(PointCloudInPtr point_cloud, PointCloudInPtr point_cloud_projected, const double & centroid_distance);
		bool findHandle(PointCloudInPtr point_cloud, PointCloudInPtr point_cloud_projected);

		void setTopIsOpenedLikelihood(PointCloudInPtr & point_cloud_top_projected);

		// Debug printing
		void printObjectType();
		void printDiscreteOrientation();


		void reestimateNormals(PointCloudInPtr ref_cloud);
		PointCloudInPtr computeNormals(PointCloudInPtr cloud);
		//Eigen::Vector4f centroid;
		PointCloudInPtr cloudWithNormals;
		PointCloudNormalPtr cloud_normals;
		static unsigned int neighbours;
	public:


	      	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	      	////////////////////////////
	      	// Object type attributes //
	      	////////////////////////////

		// Object shape type
		unsigned int object_shape_type;

		// Object symmetry type
		int object_symmetry_type;

		// Object parts number;
		int object_parts_number;

		int discrete_vertical_orientation;

		int discrete_horizontal_orientation;

		static const std::vector<Eigen::Vector3f> discrete_orientations;

		// Object pose
		Eigen::Transform<float, 3, Eigen::Affine> pose;

		// Handle pose
		Eigen::Transform<float, 3, Eigen::Affine> handle_pose;

		Eigen::Transform<float, 3, Eigen::Affine> handle_pose_world;

		// Core centered point cloud
		PointCloudInPtr centered_point_cloud;

		// Handle centered point cloud
		PointCloudInPtr handle_centered_point_cloud;

		// Handle point cloud
		PointCloudInPtr handle_point_cloud;

		// Core point cloud
		PointCloudInPtr core_point_cloud;

		// Complete point cloud
		PointCloudInPtr complete_point_cloud;

		// Complete xyz point cloud
		PointCloudInPtr point_cloud_complete_xyz_reflect;

		// Cluster projected
		PointCloudInPtr cluster_top;

		// Cluster top projected
    		PointCloudInPtr cluster_top2D;

		// Core projected
		PointCloudInPtr core2D;

		// Core top projected
   		 PointCloudInPtr core_top2D;

		// Object dimensions
		Eigen::Vector3f bounding_box;

		// Handle dimensions
		Eigen::Vector3f handle_bounding_box;

		// Object regions
		std::vector<ObjectRegion> regions;

		bool has_handle;

		bool is_tool;

		bool is_shape_type_2b_mug;

		void applyBoundingBoxOrientationRules();

		// Constructor
		ObjectDetails() //: centered_point_cloud(PointCloudInPtr (new pcl::PointCloud<pcl::PointXYZI>))
	  	{
			remove_outliers = false;
			handle_indices = pcl::PointIndices::Ptr (new pcl::PointIndices);
			body_indices = pcl::PointIndices::Ptr (new pcl::PointIndices);

			eigen_vector.resize(3);
			handle_eigen_vector.resize(3);

			core_point_cloud = PointCloudInPtr (new PointCloudIn);
			handle_point_cloud = PointCloudInPtr (new PointCloudIn);
			cluster_top = PointCloudInPtr (new PointCloudIn);
			cluster_top2D = PointCloudInPtr (new PointCloudIn);
			core2D = PointCloudInPtr (new PointCloudIn);
			core_top2D = PointCloudInPtr (new PointCloudIn);
			complete_point_cloud = PointCloudInPtr (new PointCloudIn);
			point_cloud_complete_xyz_reflect = PointCloudInPtr (new PointCloudIn);

			centered_point_cloud = PointCloudInPtr (new PointCloudIn);
			handle_centered_point_cloud = PointCloudInPtr (new PointCloudIn);

			cloud_normals = PointCloudNormalPtr (new PointCloudNormal);


			cloudWithNormals = PointCloudInPtr (new PointCloudIn);

	  	};

		// Complete object point cloud assuming 180 degrees symmetries along all directions
		//void completeObjectPointCloud(PointCloudInPtr point_cloud);
		void completeObjectPointCloudAlternative(PointCloudInPtr point_cloud);
		// Compute ellipsoid description (i.e. Bounding box by means of PCA)
		void computeObjectDetails(PointCloudInPtr point_cloud_object_details, PointCloudInPtr point_cloud);

		static void setObjectTypeThreshold(const double & _object_type_threshold)
		{
			object_type_threshold=_object_type_threshold;
		}

		static void setObjectPartsThreshold(const double & _object_parts_threshold)
		{
			D=_object_parts_threshold;
		}

		static void setTopHeightThreshold(const double & _top_height_threshold)
		{
			top_height_threshold=_top_height_threshold;
		}


		static void setDownsamplingStep(const double & _downsampling_step)
		{
			downsampling_step=_downsampling_step;
		}

		static void setMinHandlePointsPercentage(const double & _minimum_handle_points_percentage)
		{
			min_handle_points_percentage=_minimum_handle_points_percentage;
		}

		static void setInliersCircleThreshold(const double & _inliers_circle_threshold)
		{
			inliers_circle_threshold=_inliers_circle_threshold;
		}

		static void setRansacCircleThreshold(const double & _circle_ransac_distance_threshold)
		{
			circle_ransac_distance_threshold=_circle_ransac_distance_threshold;
		}

		static void setRemoveOutliers(const bool & _remove_outliers)
		{
			remove_outliers=_remove_outliers;
		}

		static void setHandleDistanceThreshold(const double & _handle_distance_threshold)
		{
			handle_distance_threshold=_handle_distance_threshold;
		}

		static void setCircleMinimumRadius(const double & _circle_minimum_radius)
		{
			circle_minimum_radius=_circle_minimum_radius;
		}

		static void setCircleMaximumRadius(const double & _circle_maximum_radius)
		{
			circle_maximum_radius=_circle_maximum_radius;
		}

		static void setCentroidDistanceThreshold(const double & _centroid_distance_threshold)
		{
			centroid_distance_threshold=_centroid_distance_threshold;
		}

		static void setVerticalHeightThreshold(const double & _vertical_height_threshold)
		{
			vertical_height_threshold=_vertical_height_threshold;
		}

		static void setHullAlpha(const double & _hull_alpha)
		{
			hull_alpha=_hull_alpha;
		}

		static void setNeighbours(const unsigned int & _neighbours)
		{
			neighbours=_neighbours;
		}

};

#endif //#ifndef ELLIPSOID_DESCRIPTION


