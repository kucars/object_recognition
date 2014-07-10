#ifndef STRUCTS
#define STRUCTS

#include <boost/serialization/shared_ptr.hpp>

#include "serialization.h"

struct transformation
{
    Eigen::Quaternion<float> rotation;
    Eigen::Translation3f translation;
    Eigen::UniformScaling<float> scale;
    transformation()
    {}

    transformation(Eigen::Quaternion<float> _rotation,Eigen::Translation3f _translation) : rotation(_rotation), translation(_translation)
    {
        scale=Eigen::UniformScaling<float> (1.000000);
    }

    transformation(Eigen::Quaternion<float> _rotation,Eigen::Translation3f _translation, Eigen::UniformScaling<float> _scale) : rotation(_rotation), translation(_translation), scale(_scale)
    {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & rotation;
        ar & translation;
        ar & scale;
    }
};

struct pose
{
    float votes;
    transformation transform;

    pose()
    {};

    pose(float _votes, const transformation & _transform) : votes(_votes), transform(_transform)
    {};

    Eigen::Affine3f getTransformation()
    {
        return transform.translation * transform.rotation * transform.scale;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & votes;
        ar & transform;
    }

    /*static bool compare(const boost::shared_ptr<pose> & a, const boost::shared_ptr<pose> & b)
        {
            return a->votes > b->votes;
        }*/

    static bool compare(const pose & a, const pose & b)
    {
        return a.votes > b.votes;
    }

};

struct poseCluster
{
public:
    //int index; //test

    int poseIndex;
    float votes;
    std::vector<pose> poses;

    poseCluster(const int & _poseIndex, const pose & newPose) : poseIndex(_poseIndex), votes(newPose.votes)
    {
        poses.push_back(newPose);
    }

    ~poseCluster()
    {
        poses.clear();
    }

    void update(const pose & newPose)
    {
        votes+=newPose.votes;
        poses.push_back(newPose);
    }

    Eigen::Quaternion<float> rotation()
    {
        Eigen::Vector4f rotationAverage (0.0, 0.0, 0.0, 0.0);

        for(size_t i=0; i<poses.size(); ++i)
        {


            /*rotationAverage(0)+=poses[i].transform.rotation.w();
                rotationAverage(1)+=poses[i].transform.rotation.x();
                rotationAverage(2)+=poses[i].transform.rotation.y();
                rotationAverage(3)+=poses[i].transform.rotation.z();*/
            //std::cout << "" <<poses[i].transform.rotation.w() << " " <<  "" <<poses[i].transform.rotation.x() << "" <<poses[i].transform.rotation.y() << "" <<poses[i].transform.rotation.z() << std::endl;
            rotationAverage(0)+=poses[i].transform.rotation.w() * poses[i].votes;
            rotationAverage(1)+=poses[i].transform.rotation.x() * poses[i].votes;
            rotationAverage(2)+=poses[i].transform.rotation.y() * poses[i].votes;
            rotationAverage(3)+=poses[i].transform.rotation.z() * poses[i].votes;
        }

        rotationAverage.normalize();

        return  Eigen::Quaternion<float>(rotationAverage(0),rotationAverage(1),rotationAverage(2),rotationAverage(3));
    }

    Eigen::Translation3f translation()
    {
        Eigen::Vector3f translationAverage (0.0, 0.0, 0.0);

        for(size_t i=0; i<poses.size(); ++i)
        {
            // Translation part
            //translationAverage += poses[i].transform.translation;

            //translationAverage(0) += poses[i].transform.translation.x();
            //translationAverage(1) += poses[i].transform.translation.y();
            //translationAverage(2) += poses[i].transform.translation.z();

            translationAverage(0)+=poses[i].transform.translation.x() * poses[i].votes;
            translationAverage(1)+=poses[i].transform.translation.y() * poses[i].votes;
            translationAverage(2)+=poses[i].transform.translation.z() * poses[i].votes;
        }

        //translationAverage /= static_cast<float> (poses.size());
        translationAverage /= votes;

        return Eigen::Translation3f(translationAverage(0),translationAverage(1),translationAverage(2));
    }

    Eigen::UniformScaling<float> scaling()
    {
        /*float scaleAverage=0;

            for(size_t i=0; i<poses.size(); ++i)
            {
                scaleAverage+=poses[i].transform.scale.factor() * poses[i].votes;
            }

            scaleAverage /= votes;*/
        return Eigen::UniformScaling<float>(poses[0].transform.scale.factor());
    }

    void normalizeVotes(const float & normalizeFactor)
    {
        votes*=normalizeFactor;
        for(size_t i=0; i<poses.size(); ++i)
        {
            poses[i].votes*=normalizeFactor;
        }
    }

    static bool compare(const poseCluster & a, const poseCluster & b)
    {
        return a.votes > b.votes;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

        float quaternNorm, quaternNormInv;
};


struct cluster
{



    std::vector<int> matches_per_feature;

    double voting_time;
    double clustering_time;
    pose meanPose;
    std::vector<pose> poses;


    std::vector<double> scene_to_global_time;
    std::vector<double> reset_accumulator_time;
    std::vector<double> ppf_time;
    std::vector<double> hash_time;
    std::vector<double> matching_time;
    std::vector<double> get_best_peak_time;

    cluster()
    {};

    cluster(const pose & _meanPose, const std::vector<pose> & _poses) : meanPose(_meanPose), poses(_poses)
    {};

    ~cluster()
    {
        poses.clear();
    }


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & poses;
        ar & meanPose;
        ar & voting_time;
        ar & clustering_time;
        ar & matches_per_feature;

        ar & scene_to_global_time;
        ar & reset_accumulator_time;
        ar & ppf_time;
        ar & hash_time;
        ar & matching_time;
        ar & get_best_peak_time;

    }

    void normalizeVotes(const float & normalizeFactor)
    {
        meanPose.votes*=normalizeFactor;
        for(size_t i=0; i<poses.size(); ++i)
        {
            poses[i].votes*=normalizeFactor;
        }
    }

    static bool compare(const cluster & a, const cluster & b)
    {
        return a.meanPose.votes > b.meanPose.votes;
    }
};

#endif //#ifndef STRUCTS
