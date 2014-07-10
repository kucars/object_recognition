#ifndef POINT_PAIR_SI
#define POINT_PAIR_SI

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>

#include "objectrecognition/serialization.h"
#include "objectrecognition/point_pair.h"

class pointPairSI : public pointPair
{
	public:
	// Variables
		float distance;		
		float distanceInverted;
		static unsigned int maxHash;

	// Constructors

		// Parameters initializer
		pointPairSI(unsigned int _angleBins) : pointPair(_angleBins)
		{
			maxHash=ceil(angleBins/2)*ceil(angleBins/2)*ceil(angleBins/2);
		};

		pointPairSI(pcl::PointNormal & _primaryPoint, pcl::PointNormal & _secondaryPoint, int _id=0) : pointPair(_primaryPoint,_secondaryPoint,_id)
   		{
			distance=(_secondaryPoint.getVector3fMap()-_primaryPoint.getVector3fMap()).norm();
			distanceInverted=1/distance;
   		}

	// Destructors
		~pointPairSI()
		{};

	// Public methods
		const unsigned int getHash(const pcl::PointNormal & secondaryPoint);

	private:
	// Private methods
		static const Eigen::Vector3f SIPPF(const pcl::PointNormal & pointOne,const pcl::PointNormal & pointTwo);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{	
        	ar & boost::serialization::base_object<pointPair>(*this);
			ar & distance;
			ar & distanceInverted;
		}

};

#endif //#ifndef POINT_PAIR_SI
