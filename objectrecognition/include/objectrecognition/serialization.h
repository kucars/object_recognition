#ifndef SERIALIZATION
#define SERIALIZATION

#include <pcl/PointIndices.h>
#include <pcl/PCLHeader.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <std_msgs/Header.h>
// std_msgs::Header serialization
namespace boost {
namespace serialization {
	template<class Archive>
	void serialize(Archive & ar,std_msgs::Header & g, const unsigned int version)
	{
	    ar & g.seq;
	    //ar & g.stamp;
	    ar & g.frame_id;
	}
} // namespace serialization
} // namespace boost

// pcl::PointXYZ and pcl::PointNormal serialization
namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
		{
			ar & g.seq;
			ar & g.stamp;
			ar & g.frame_id;
		}

		template<class Archive>
		void serialize(Archive & ar, pcl::PointXYZ & g, const unsigned int version)
		{
			ar & g.x;
			ar & g.y;
			ar & g.z;
		}

		template<class Archive>
		void serialize(Archive & ar, pcl::PointNormal & g, const unsigned int version)
		{
			ar & g.x;
			ar & g.y;
			ar & g.z;
			ar & g.normal[0];
			ar & g.normal[1];
			ar & g.normal[2];
		}

	} // namespace serialization
} // namespace boost


// Eigen::Quaternion, Eigen:Translation serialization , Eigen::UniformScaling serialization
namespace boost {
namespace serialization {
	template<class Archive>
	void serialize(Archive & ar,Eigen::Quaternion<float> & g, const unsigned int version)
	{
	    ar & g.w();
	    ar & g.x();
	    ar & g.y();
	    ar & g.z();
	}
	template<class Archive>
	void serialize(Archive & ar, Eigen::Translation<float, 3> & g, const unsigned int version)
	{
	    ar & g.x();
	    ar & g.y();
	    ar & g.z();	
	}

	template<class Archive>
	void serialize(Archive & ar, Eigen::UniformScaling<float> & g, const unsigned int version)
	{
	    ar & g.factor();
	}
} // namespace serialization
} // namespace boost

// pcl::PointIndices serialization
namespace boost {
namespace serialization {
	template<class Archive>
	void serialize(Archive & ar, pcl::PointIndices & g, const unsigned int version)
	{
	    ar & g.indices;
	    ar & g.header;
	}
} // namespace serialization
} // namespace boost

// pcl::Pointcloud serialization
namespace boost {
namespace serialization {
	template<class Archive>
	void serialize(Archive & ar, pcl::PointCloud<pcl::PointNormal> & g, const unsigned int version)
	{
	    ar & g.header;
	    ar & g.points;
	    ar & g.height;
	    ar & g.width;
	    ar & g.is_dense;
	}

	template<class Archive>
	void serialize(Archive & ar, pcl::PointCloud<pcl::PointXYZ> & g, const unsigned int version)
	{
	    ar & g.header;
	    ar & g.points;
	    ar & g.height;
	    ar & g.width;
	    ar & g.is_dense;
	}

} // namespace serialization
} // namespace boost

// Eigen::Affine3f serialization
namespace boost {
namespace serialization {

	template<class Archive>
	void serialize(Archive & ar, Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> & g, const unsigned int version)
	{

	}



} // namespace serialization
} // namespace boost
#endif //#ifndef SERIALIZATION
