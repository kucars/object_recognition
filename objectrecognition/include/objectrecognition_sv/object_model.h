#ifndef OBJECT_MODEL_SV
#define OBJECT_MODEL_SV

#include "objectrecognition_sv/point_pair.h"
#include "objectrecognition/object_model.h"


class objectModelSV : public objectModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Objects
    std::vector<std::vector<pointPairSV> > hashTable;
    // Constructors

    // Parameters initializer
    objectModelSV(int _angleBins, int _distanceBins, float _radius, float _neighbours, bool _radiusSearch) : objectModel(_angleBins, _distanceBins, _radius, _neighbours, _radiusSearch)
    {
        // Initialize parameters
        pointPairSV(_angleBins,_distanceBins);
    }

    objectModelSV(int _objectId, std::tr1::tuple<pointCloudPointNormalPtr, float, float, shape_msgs::MeshPtr> & _modelData, bool _symmetric) : objectModel(_objectId, _modelData, _symmetric)
    {
        //std::cout << "YAHH" << std::endl;

        std::cout <<"DIST BINS:" << objectModel::distanceBins << " ANGLE BINS:" << pointPair::angleBins << std::endl;
        pointPairSV(pointPair::angleBins, objectModel::distanceBins); // TIRAR

        train();
        int i = 0;
        for(pcl::PointCloud<pcl::PointNormal>::iterator it=modelCloud->begin(); it< modelCloud->end(); it++)
        {
            Eigen::Vector3f modelPoint=it->getVector3fMap();
            Eigen::Vector3f modelNormal=it->getNormalVector3fMap ();
            Eigen::Vector3f cross=modelNormal.cross (Eigen::Vector3f::UnitX ()).normalized ();
            Eigen::AngleAxisf rotationPointToGlobal(acosf (modelNormal.dot (Eigen::Vector3f::UnitX ())), cross);
            ++i;
            if (isnan(cross[0]))
            {
                std::cout << "i: "<<  i << std::endl;
                rotationPointToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
            }

            Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> transformSurfletToGlobal = Eigen::Translation3f( rotationPointToGlobal * ((-1) * modelPoint)) * rotationPointToGlobal;
            modelCloudTransformations.push_back(transformSurfletToGlobal);
            //std::cout << modelCloudTransformations.back().matrix() << modelNormal.cross (Eigen::Vector3f::UnitX ()).normalized() << " " << modelNormal << std::endl;
        }
    }

    objectModelSV()
    {
        maxModelDist=0;
        maxModelDistSquared=0;
        modelCloud=pointCloudPointNormalPtr (new pointCloudPointNormal);
    }
    // Destructor
    ~objectModelSV()
    {}
    // Public methods
    int computeIndexDebug(const Eigen::Vector4f& feature);
    virtual void seeHashTableEntropy();
    transformation modelToScene(const pcl::PointNormal & pointModel, const Eigen::Affine3f & transformSceneToGlobal, const float alpha);
    transformation modelToScene(const int modelPointIndex, const Eigen::Affine3f & transformSceneToGlobal, const float alpha);

private:
    void train();

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<objectModel>(*this);
        ar & hashTable;
    }
};

#endif //#ifndef OBJECT_MODEL
