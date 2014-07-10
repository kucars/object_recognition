#ifndef POINT_PAIR_SV
#define POINT_PAIR_SV

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>

#include "objectrecognition/serialization.h"
#include "objectrecognition/point_pair.h"

class pointPairSV : public pointPair
{
	public:
	// Parameters
		static unsigned int distanceBins;
		static unsigned int maxHash;

	// Variables
//		Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> transformToGlobal;

		int weight;
		pcl::PointNormal otherPoint; //reference point

	// Constructors

		// Parameters initializer
		pointPairSV(const unsigned int & _angleBins, const unsigned int & _distanceBins) : pointPair(_angleBins)
		{
			distanceBins=_distanceBins;

			maxHash=distanceBins*featureAngleBins*featureAngleBins*featureAngleBins;
			std::cout << distanceBins << std::endl;
			std::cout << "maxHash:" << maxHash << std::endl;
		};

		pointPairSV(pcl::PointNormal & _primaryPoint, pcl::PointNormal & _secondaryPoint,  const Eigen::Affine3f & _transformToGlobal, int _id=0) : pointPair(_primaryPoint,_secondaryPoint, _transformToGlobal,_id), weight(1), otherPoint(_secondaryPoint)
   		{
/*
			Eigen::Vector3f modelRefPoint=_primaryPoint.getVector3fMap();
			Eigen::Vector3f modelRefNormal=_primaryPoint.getNormalVector3fMap ();

			// Get transformation from model local frame to global frame
			Eigen::AngleAxisf rotationModelToGlobal(acosf (modelRefNormal.dot (Eigen::Vector3f::UnitX ())), modelRefNormal.cross (Eigen::Vector3f::UnitX ()).normalized ());
			transformToGlobal = Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelRefPoint)) * rotationModelToGlobal;
*/
		};

		pointPairSV()
		{};

	// Destructor
		~pointPairSV()
		{};

	    friend std::ostream& operator<<(std::ostream& os, const pointPairSV& ppf);

	// Public methods
		const unsigned int getHash(const pcl::PointNormal & secondaryPoint, const float & distanceStepInverted);

	private:
	// Private methods
		friend class objectModelSV;
		static const Eigen::Vector4f PPF(const pcl::PointNormal & pointOne,const pcl::PointNormal & pointTwo);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{	
        	ar & boost::serialization::base_object<pointPair>(*this);
//			ar & transformToGlobal;
			ar & weight;
			//ar & otherPoint;
		}

//		Eigen::Vector4f ppf;
//		Eigen::Vector4d discrete_ppf;

};

#endif //#ifndef POINT_PAIR_SV
