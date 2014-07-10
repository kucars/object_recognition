#ifndef OBJECT_MODEL
#define OBJECT_MODEL

#include <ctime>

#include <tr1/tuple>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl_visualization/point_cloud_handlers.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>

#include <boost/thread/thread.hpp>
#include <boost/numeric/conversion/cast.hpp> 

#include <tf/tf.h>

#include <pcl_ros/transforms.h>

//#include "unordered_map_serialization.hpp"

#include "defines.h"
#include "serialization.h"
#include "structs.h"
#include "point_pair.h"
#include <shape_msgs/Mesh.h>

inline bool equalFloat(float a, float b)
{
    return fabs(a - b) < EPSILON;
}

inline bool equalFloat(float a, float b, float epsilon)
{
    return fabs(a - b) < epsilon;
}

class objectModel
{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> pointCloudPointXYZ;
		typedef pointCloudPointXYZ::Ptr pointCloudPointXYZPtr;
		typedef pcl::PointCloud<pcl::PointNormal> pointCloudPointNormal;
		typedef pointCloudPointNormal::Ptr pointCloudPointNormalPtr;

	// Global parameters (for surface normals computation)
		static bool radiusSearch;
		static float radius;
		static float neighbours;
		
		static unsigned int distanceBins;
		unsigned int subsampleBins;

	// Variables
		unsigned id;
		unsigned int dbId;
		float totalSurfaceArea;
		float maxModelDist;
		float maxModelDistSquared;

        shape_msgs::MeshPtr modelMesh;
		pointCloudPointNormalPtr modelCloud;
		pointCloudPointNormalPtr modelCloudOriginal;

		std::vector<Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> > modelCloudTransformations;

		bool symmetric;

		float distanceStep;
		float distanceStepInverted;
		float halfDistanceStepSquared;
		float subsampleStep;
	// Constructors

		// Parameters initializer
		objectModel(int _angleBins, int _distanceBins, float _radius, float _neighbours, bool _radiusSearch)
		{
			distanceBins=_distanceBins;

			// Initialize parameters
			pointPair(angleBins);

			radius=_radius;
			neighbours=_neighbours;
			radiusSearch=_radiusSearch;
		}

        objectModel(int _dbId, std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr > & _modelData, bool _symmetric) : id(++idNext), dbId(_dbId), totalSurfaceArea(std::tr1::get<1>(_modelData)), maxModelDist(sqrt(std::tr1::get<2>(_modelData))), maxModelDistSquared(std::tr1::get<2>(_modelData)), modelMesh(std::tr1::get<3>(_modelData)), modelCloudOriginal(std::tr1::get<0>(_modelData)), symmetric(_symmetric), distanceStep(maxModelDist/distanceBins), distanceStepInverted(distanceBins/maxModelDist), halfDistanceStepSquared((distanceStep/2)*(distanceStep/2))
		{
			//distanceStep=(1.0)/2.0; // TIRAR
			//distanceStep=(1.0)/2.0; // TIRAR
			//distanceStepInverted=1.0/distanceStep;// TIRAR
			//halfDistanceStepSquared=(distanceStep/2.0)*(distanceStep/2.0);// TIRAR
			//distanceBins=ceil(maxModelDist/distanceStep);// TIRAR*/
			/*if(distanceBins%2==0)
			{
				subsampleBins=floor(distanceBins/2.0)-1; // o -1 é para o caso da divisão ser par
			}
			else
			{
				subsampleBins=floor(distanceBins/2.0); // o -1 é para o caso da divisão ser par
			}
			std::cout << "subsampling bins: " << subsampleBins << std::endl;
			subsampleStep=maxModelDist/subsampleBins;*/
			//subsampleStep=distanceStep;
			//distanceBins=ceil(maxModelDist/distanceStep)+1;// TIRAR
			//distanceStep=maxModelDist/distanceBins;
			

			std::cout << "dist bins:" << distanceBins << std::endl;
//			std::cout << "YEAH" << std::endl;
			modelCloud=pointCloudPointNormalPtr (new pointCloudPointNormal);
			*modelCloud=*modelCloudOriginal;

			// Get object radius
			float radius;
			float maxSquaredDistance=0.0;

			for(pcl::PointCloud<pcl::PointNormal>::iterator mr=modelCloud->begin(); mr<modelCloud->end(); ++mr)
			{		// Distance to rotation axis
					float squaredDistance=(mr->x)*(mr->x)+(mr->y)*(mr->y);
					if(squaredDistance>maxSquaredDistance)
					{
						maxSquaredDistance=squaredDistance;
						radius=squaredDistance;
					}
			}

			radius=sqrt(radius);
			
			/*float alpha_step_=acos(1-0.5*(distanceStep/radius)*(distanceStep/radius));	
			float autoAngleBins=2*PI/alpha_step_;
			unsigned int nyquistAngleBins=ceil(autoAngleBins); // é preciso o +1?
			std::cout << "nyquist bins: " << nyquistAngleBins << std::endl;
			pointPair recompute(nyquistAngleBins);*/

			cloudWithNormals=pointCloudPointNormalPtr (new pointCloudPointNormal);
			cloud_normals=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
			std::cout << "Distance step:" << distanceStep << std::endl;
			if(symmetric)
				std::cout << " Object with db id=" << dbId << " is symmetric around z axis" << std::endl;
			else
				std::cout << " Object with db id=" << dbId << " is not symmetric" << std::endl;
		}

		objectModel()
		{
			cloudWithNormals=pointCloudPointNormalPtr (new pointCloudPointNormal);
			cloud_normals=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		}

	// Destructor
		virtual ~objectModel() {}

	// Public methods
		static void changeParameters(float _angleBins, float _distanceBins, float _radius, double _neighbours, bool _radius_search);
		pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		pcl::PointCloud<pcl::PointNormal>::Ptr computeSceneNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud);
		virtual void seeHashTableEntropy()=0;
		static void quaternionToEuler(const Eigen::Quaternion<float> & q, float & roll, float & pitch, float & yaw);
		static void eulerToQuaternion(Eigen::Quaternion<float> & q, const float & roll, const float & pitch, const float & yaw);

		void computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloudCompareNormals);
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{	
			ar & id;
			ar & dbId;
			ar & maxModelDist;

			ar & maxModelDistSquared;
			ar & symmetric;

			ar & radiusSearch;
			ar & radius;
			ar & neighbours;

			ar & distanceBins;
			ar & distanceStep;
			ar & distanceStepInverted;
			ar & halfDistanceStepSquared;

			ar & totalSurfaceArea;

			ar & modelCloud;
			ar & modelCloudOriginal;
		}

	protected:
		virtual void train()=0;
		static int idNext;
		Eigen::Vector4f centroid;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
};

#endif //#ifndef OBJECT_MODEL
