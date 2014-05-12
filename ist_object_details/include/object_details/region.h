#include <Eigen/Eigen>
#define EIGEN_DONT_ALIGN
#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE
class ObjectRegion
{
	public:
		ObjectRegion();

		ObjectRegion(const int & _id, const int & _type, const Eigen::Transform<float, 3, Eigen::Affine> & _pose, const Eigen::Vector3f & _bounding_box, double & _likelihood) : id(_id), type(_type), pose(_pose), bounding_box(_bounding_box), likelihood(_likelihood)
		{};

		// Region pose
		Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> pose;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Region id
		int id;

		// Region type
		int type;

		// Region dimensions
		Eigen::Vector3f bounding_box;

		// Region likelihood
		double likelihood;

};
