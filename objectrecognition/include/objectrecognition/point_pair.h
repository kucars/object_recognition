#ifndef POINT_PAIR
#define POINT_PAIR

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>

#include "objectrecognition/defines.h"
#include "objectrecognition/serialization.h"

class pointPair
{
	public:
	// Parameters
		static unsigned int angleBins;
		static float angleStep;
		static float angleStepInverted;
		static float angleStepCos;

		static const unsigned int cosResolution=10000;
		static std::vector<unsigned int> lutCosToAngle;

	// Variables
		pcl::PointNormal referencePoint; //reference point
		int id;
		float alpha;


	// Constructors

		// Parameters initializer
		pointPair(const unsigned int & _angleBins)
		{
			angleBins=_angleBins;
			angleStep=2.0*PI/angleBins;
			angleStepInverted=1.0/angleStep; // optimization
			angleStepCos=cos(angleStep/2.0); // quaternion difference formula
			//if(lutCosToAngle.size()==0)
			{
				lutCosToAngle.resize(cosResolution);

				for(size_t i=0; i<lutCosToAngle.size(); ++i)
				{
					lutCosToAngle[i]=round(acos( ((2.0/static_cast<float>(cosResolution)) * static_cast<float>(i)) -1.0)/angleStep);
					//std::cout << "bin: " << i << ":" << lutCosToAngle[i] << std::endl;
				}
			}
			featureAngleBins=ceil(angleBins/2.0);
			std::cout << "FEATURE ANGLE BINS:" <<  featureAngleBins << std::endl;
		};

		pointPair(const pcl::PointNormal & _primaryPoint, const pcl::PointNormal & _secondaryPoint, const Eigen::Affine3f & _transformToGlobal, int _id=0) : referencePoint(_primaryPoint), id(_id),alpha(getRotation(_secondaryPoint,_transformToGlobal))
		{
			//referencePoint.getNormalVector3fMap().normalize();
		};

		pointPair()
		{};

	// Destructor
		virtual ~pointPair()
		{};

	private:

		float getRotation(const pcl::PointNormal & secondaryPoint, const Eigen::Affine3f & transformToGlobal);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{	
			ar & id;
			ar & referencePoint;
			ar & alpha;
			ar & angleBins;
			ar & angleStep;
			ar & angleStepInverted;
			ar & angleStepCos;
			ar & featureAngleBins;
		}

	protected:
		static unsigned int featureAngleBins;
};

#endif //#ifndef POINT_PAIR
