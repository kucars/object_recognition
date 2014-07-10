#ifndef OBJECT_MODEL_SI
#define OBJECT_MODEL_SI

#include "objectrecognition_si/point_pair.h"
#include "objectrecognition/object_model.h"


class objectModelSI : public objectModel
{
	public:
      	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// Objects
		std::vector<std::vector<pointPairSI> > hashTable;

	// Constructors

		objectModelSI(int _objectId, std::tr1::tuple<pointCloudPointNormalPtr, float, float> & _modelData, bool _symmetric) : objectModel(_objectId, _modelData, _symmetric)
   		{
			train();
		}

		objectModelSI() 
		{
			maxModelDist=0;
			maxModelDistSquared=0;
			modelCloud=pointCloudPointNormalPtr (new pointCloudPointNormal);;
		}
	// Destructor
		~objectModelSI()
		{}
	// Public methods
		virtual void seeHashTableEntropy();
		boost::shared_ptr < transformation > modelToScene(const pcl::PointNormal & pointModel, const Eigen::Affine3f & transformSceneToGlobal, const float alpha, const float scale);

	private:
		void train();
		void train(pointCloudPointXYZPtr _modelCloud);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{	
        		ar & boost::serialization::base_object<objectModel>(*this);
			ar & hashTable;
		}
};

#endif //#ifndef OBJECT_MODEL
