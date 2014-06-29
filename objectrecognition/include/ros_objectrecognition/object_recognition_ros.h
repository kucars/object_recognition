#include <vector>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database_msgs/DatabaseModelPoseList.h>
#include <household_objects_database_msgs/GetModelList.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <bottomup_msgs/Region.h>
#include <bottomup_msgs/bottomup_msg.h>
#include <bottomup_msgs/Object.h>

#include "objectrecognition/models.h"


#define MODEL_SET "REDUCED_MODEL_SET" //leave empty("") for all models



template <class objectModelT>
class objectRecognitionRos
{
public:
    // variables
    static ros::Publisher marker_pub;
    // methods
    template <class clusterT>
    static bottomup_msgs::Region fillRegionMsg(int const regionId, std::vector< std::vector<clusterT> > const hypotheses, models<objectModelT> const modelsLibrary, unsigned int const hypotheses_per_model, int const poseAngleBins, pcl::PointXYZ & min_pt, pcl::PointXYZ & max_pt,std::string processing_frame)
    {
        sensor_msgs::PointCloud2 graspablePointsMsg;
        bottomup_msgs::Hypothesis msg_hypothesis;
        bottomup_msgs::Region msg_region;

        // Region id
        msg_region.id=regionId;
        // for each model...
        for(size_t m = 0; m < hypotheses.size(); ++m)
        {
            // Send top N hypotheses each for each model, for each cluster
            if((unsigned int)hypotheses[m].size()<hypotheses_per_model)
            {

                // Create artificial hypotheses with zero probability
                ROS_INFO("ERROR! Incoherent hypothesis number: 	%zu<%u",hypotheses[m].size(),hypotheses_per_model);
                //exit(0);
            }
            // Send top N hypotheses each for each model, for each cluster
            for(size_t c = 0; c < hypotheses[m].size() && (int)msg_region.hypothesis.size() < hypotheses_per_model*(m+1); ++c)
            {
                msg_hypothesis.cont=0;
                msg_hypothesis.likelihood_gfd=0;
                msg_hypothesis.match=0;
                msg_hypothesis.obj_id=modelsLibrary.objectModels[m]->id+1;
                msg_hypothesis.db_id=modelsLibrary.objectModels[m]->dbId;

                /*pcl::getTranslationAndEulerAngles (hypotheses[m][c]->meanPose->getTransformation(),
                                       msg_hypothesis.x,
                                       msg_hypothesis.y,
                                       msg_hypothesis.z,
                                       msg_hypothesis.rol,
                                       msg_hypothesis.pit,
                                       msg_hypothesis.yaw);
                    */
                quaternionToEuler(hypotheses[m][c].meanPose.transform.rotation, msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw);
                //std::cout << "antes2: " << msg_hypothesis.yaw << std::endl;

                //pcl::getEulerAngles (hypothesis[m][c]->meanPose->getTransformation(),
                //				   roll,
                //				   pitch,
                //ma				   yaw);
                msg_hypothesis.x=hypotheses[m][c].meanPose.transform.translation.x();
                msg_hypothesis.y=hypotheses[m][c].meanPose.transform.translation.y();
                msg_hypothesis.z=hypotheses[m][c].meanPose.transform.translation.z();
                msg_hypothesis.pose.header.frame_id=processing_frame;
                msg_hypothesis.pose.pose.position.x=hypotheses[m][c].meanPose.transform.translation.x();
                msg_hypothesis.pose.pose.position.y=hypotheses[m][c].meanPose.transform.translation.y();
                msg_hypothesis.pose.pose.position.z=hypotheses[m][c].meanPose.transform.translation.z();

                msg_hypothesis.pose.pose.orientation.w=hypotheses[m][c].meanPose.transform.rotation.w();
                msg_hypothesis.pose.pose.orientation.x=hypotheses[m][c].meanPose.transform.rotation.x();
                msg_hypothesis.pose.pose.orientation.y=hypotheses[m][c].meanPose.transform.rotation.y();
                msg_hypothesis.pose.pose.orientation.z=hypotheses[m][c].meanPose.transform.rotation.z();

                msg_hypothesis.likelihood=hypotheses[m][c].meanPose.votes;
                msg_hypothesis.p=objectRecognitionRos<objectModelT>::eulerToIndex(msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw,poseAngleBins);

                //pcl::PointCloud<pcl::PointNormal>::Ptr graspablePoints(new pcl::PointCloud<pcl::PointNormal>);
                //graspablePoints=modelsLibrary.objectModels[m]->sceneGraspablePoints(hypotheses[m][c]->meanPose->getTransformation());

                // convert graspable point cloud to a ROS message
                //pcl::toROSMsg(*graspablePoints, graspablePointsMsg);
                //msg_hypothesis.graspable_points=graspablePointsMsg;
                msg_region.hypothesis.push_back(msg_hypothesis);


                //msg_region.scene_cloud=convertedCloud;
            }
        }

        //region limits
        //ROS_INFO("msg_region id:%d",msg_region.id);
        msg_region.minimum_point.x=min_pt.x;
        msg_region.minimum_point.y=min_pt.y;
        msg_region.minimum_point.z=min_pt.z;
        msg_region.maximum_point.x=max_pt.x;
        msg_region.maximum_point.y=max_pt.y;
        msg_region.maximum_point.z=max_pt.z;

        return msg_region;
    }

    template <class clusterT>
    static bottomup_msgs::Region fillRegionMsg(int const regionId, std::vector< std::vector<boost::shared_ptr<clusterT> > > const hypotheses, models<objectModelT> const modelsLibrary, int const hypotheses_per_model, int const poseAngleBins, std::string processing_frame)
    {
        sensor_msgs::PointCloud2 graspablePointsMsg;
        bottomup_msgs::Hypothesis msg_hypothesis;
        bottomup_msgs::Region msg_region;

        // Region id
        msg_region.id=regionId;

        for(size_t m = 0; m < hypotheses.size(); ++m)
        {
            // Send top N hypotheses each for each model, for each cluster
            for(size_t c = 0; c < hypotheses[m].size() && (int)msg_region.hypothesis.size() < hypotheses_per_model*(m+1); ++c)
            {
                msg_hypothesis.cont=0;
                msg_hypothesis.likelihood_gfd=0;
                msg_hypothesis.match=0;
                msg_hypothesis.obj_id=modelsLibrary.objectModels[m]->id+1;
                msg_hypothesis.db_id=modelsLibrary.objectModels[m]->dbId;

                /*pcl::getTranslationAndEulerAngles (hypotheses[m][c]->meanPose->getTransformation(),
                                       msg_hypothesis.x,
                                       msg_hypothesis.y,
                                       msg_hypothesis.z,
                                       msg_hypothesis.rol,
                                       msg_hypothesis.pit,
                                       msg_hypothesis.yaw);
                    */
                quaternionToEuler(hypotheses[m][c]->meanPose->transform->rotation, msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw);
                //std::cout << "antes3: " << msg_hypothesis.yaw << std::endl;

                //pcl::getEulerAngles (hypothesis[m][c]->meanPose->getTransformation(),
                //				   roll,
                //				   pitch,
                //ma				   yaw);
                msg_hypothesis.x=hypotheses[m][c]->meanPose->transform->translation.x();
                msg_hypothesis.y=hypotheses[m][c]->meanPose->transform->translation.y();
                msg_hypothesis.z=hypotheses[m][c]->meanPose->transform->translation.z();
                msg_hypothesis.pose.header.frame_id=processing_frame;
                msg_hypothesis.pose.pose.position.x=hypotheses[m][c]->meanPose->transform->translation.x();
                msg_hypothesis.pose.pose.position.y=hypotheses[m][c]->meanPose->transform->translation.y();
                msg_hypothesis.pose.pose.position.z=hypotheses[m][c]->meanPose->transform->translation.z();

                msg_hypothesis.pose.pose.orientation.w=hypotheses[m][c]->meanPose->transform->rotation.w();
                msg_hypothesis.pose.pose.orientation.x=hypotheses[m][c]->meanPose->transform->rotation.x();
                msg_hypothesis.pose.pose.orientation.y=hypotheses[m][c]->meanPose->transform->rotation.y();
                msg_hypothesis.pose.pose.orientation.z=hypotheses[m][c]->meanPose->transform->rotation.z();

                msg_hypothesis.likelihood=hypotheses[m][c]->meanPose->votes;
                msg_hypothesis.p=objectRecognitionRos<objectModelT>::eulerToIndex(msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw,poseAngleBins);

                //pcl::PointCloud<pcl::PointNormal>::Ptr graspablePoints(new pcl::PointCloud<pcl::PointNormal>);
                //graspablePoints=modelsLibrary.objectModels[m]->sceneGraspablePoints(hypotheses[m][c]->meanPose->getTransformation());

                // convert graspable point cloud to a ROS message
                //pcl::toROSMsg(*graspablePoints, graspablePointsMsg);
                //msg_hypothesis.graspable_points=graspablePointsMsg;
                msg_region.hypothesis.push_back(msg_hypothesis);

                //region id
                msg_region.id=0;
                //region limits
                msg_region.minimum_point.x=0;
                msg_region.minimum_point.y=0;
                msg_region.minimum_point.z=0;
                msg_region.maximum_point.x=1.0;
                msg_region.maximum_point.y=1.0;
                msg_region.maximum_point.z=1.0;
                //msg_region.scene_cloud=convertedCloud;
            }
        }

        return msg_region;
    }

    template <class clusterT>
    static bottomup_msgs::Region fillRegionMsg(int const regionId, std::vector< std::vector<clusterT > > const hypotheses, int const hypotheses_per_model, int const poseAngleBins, std::string processing_frame)
    {
        static int iteration=0;
        float gridSize=0.5; // in meters
        int gridXBins=2;
        int gridYBins=2;
        int gridZBins=5;
        float xCenter=0.5; //martelado para bater certo com as deteccoes que tem uma translacao de 40 cm em x
        float gridXStep = static_cast<float> (gridSize/gridXBins);
        float gridYStep = static_cast<float> (gridSize/gridYBins);
        float gridZStep = static_cast<float> (gridSize/gridZBins);

        // position offsets
        float positionXOffset=gridXBins*gridXStep*0.5;
        float positionYOffset=gridYBins*gridYStep*0.5;
        float positionZOffset=gridZBins*gridZStep*0.5;



        sensor_msgs::PointCloud2 graspablePointsMsg;
        bottomup_msgs::Hypothesis msg_hypothesis;
        bottomup_msgs::Region msg_region;



        // Region id
        msg_region.id=regionId;
        for(size_t m = 0; m < hypotheses.size(); ++m)
        {


            //std::cout << "size:" << hypotheses[m].size() << std::endl;
            // Send top N hypotheses each for each model, for each cluster
            for(int c = 0; c < (int)hypotheses[m].size() && (int)msg_region.hypothesis.size() < hypotheses_per_model*(m+1); ++c)
            {

                //if(m==2)
                //	std::cout << "ola:" << msg_region.hypothesis.size()<< std::endl;
                msg_hypothesis.cont=0;
                msg_hypothesis.likelihood_gfd=0;
                msg_hypothesis.match=0;
                msg_hypothesis.obj_id=m+1;
                msg_hypothesis.db_id=m+1;


                quaternionToEuler(hypotheses[m][c]->meanPose->transform->rotation, msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw);

                msg_hypothesis.x=hypotheses[m][c]->meanPose->transform->translation.x();
                msg_hypothesis.y=hypotheses[m][c]->meanPose->transform->translation.y();
                msg_hypothesis.z=hypotheses[m][c]->meanPose->transform->translation.z();
                /*if(iteration==35 && c==0)
                    {
                        std::cout << "iteration:35 object:" << m << " votes:" << hypotheses[m][c]->meanPose->votes << std::endl;

                    }*/

                //std::cout << "x:" << msg_hypothesis.x << " y:" << msg_hypothesis.y << " z:" << msg_hypothesis.z << std::endl;


                if( (fabs(msg_hypothesis.x-xCenter) > gridSize/2) ||
                        (fabs(msg_hypothesis.y) > gridSize/2) ||
                        (fabs(msg_hypothesis.z) > gridSize/2) )
                {
                    //exit(0);
                    continue;
                }

                msg_hypothesis.pose.header.frame_id=processing_frame;
                msg_hypothesis.pose.pose.position.x=hypotheses[m][c]->meanPose->transform->translation.x();
                msg_hypothesis.pose.pose.position.y=hypotheses[m][c]->meanPose->transform->translation.y();
                msg_hypothesis.pose.pose.position.z=hypotheses[m][c]->meanPose->transform->translation.z();

                msg_hypothesis.pose.pose.orientation.w=hypotheses[m][c]->meanPose->transform->rotation.w();
                msg_hypothesis.pose.pose.orientation.x=hypotheses[m][c]->meanPose->transform->rotation.x();
                msg_hypothesis.pose.pose.orientation.y=hypotheses[m][c]->meanPose->transform->rotation.y();
                msg_hypothesis.pose.pose.orientation.z=hypotheses[m][c]->meanPose->transform->rotation.z();

                msg_hypothesis.likelihood=hypotheses[m][c]->meanPose->votes;
                //msg_hypothesis.p=objectRecognitionRos<objectModelT>::eulerToIndex(msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw,poseAngleBins);
                msg_hypothesis.p=objectRecognitionRos<objectModelT>::poseToIndex(msg_hypothesis.rol, msg_hypothesis.pit, msg_hypothesis.yaw, poseAngleBins, msg_hypothesis.x, msg_hypothesis.y, msg_hypothesis.z, gridXBins, gridYBins, gridZBins, gridXStep, gridYStep, gridZStep, positionXOffset, positionYOffset, positionZOffset);

                //std::cout << "hypotheses: " << msg_hypothesis.p << std::endl;
                msg_region.hypothesis.push_back(msg_hypothesis);

                //region id
                msg_region.id=0;
                //region limits
                msg_region.minimum_point.x=0;
                msg_region.minimum_point.y=0;
                msg_region.minimum_point.z=0;
                msg_region.maximum_point.x=1.0;
                msg_region.maximum_point.y=1.0;
                msg_region.maximum_point.z=1.0;
                //msg_region.scene_cloud=convertedCloud;
            }
        }
        iteration++;
        return msg_region;
    }
    /*static bool loadModels(bool trainMode, bool testMode, boost::shared_ptr<models<objectModelT> > modelsData, float scale,ros::NodeHandle n_,std::string modelsFile)
        {
            if(testMode)
            {
                loadModelsTest(modelsData, scale, n_);
                return true;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudDownSampled;
            pcl::PointCloud<pcl::PointNormal>::Ptr modelCloudNormals;

            // Load data, if available
            ROS_INFO("Loading file %s",modelsFile.c_str());
            modelsData->loadData(modelsFile.c_str());
            ROS_INFO("Done");

            typename std::vector<boost::shared_ptr<objectModelT> >::iterator it;
            if(trainMode)
            {

                // Connect do household objects database

                // Service client for getting the objects list from the database
                ros::ServiceClient get_model_list_srv_client_ = n_.serviceClient<household_objects_database_msgs::GetModelList> (GET_MODEL_LIST_SRV, true);

                household_objects_database_msgs::GetModelList get_model_list_srv_;

                while ( !ros::service::waitForService(GET_MODEL_LIST_SRV, ros::Duration(2.0)) && n_.ok() )
                {
                    ROS_INFO("Waiting for %s service to come up", GET_MODEL_LIST_SRV);
                }
                if (!n_.ok())
                    exit(0);

                // Check if there are new models to train
                ROS_INFO("Models already trained: %d",(int) modelsData->objectModels.size());
                for(it=modelsData->objectModels.begin(); it<modelsData->objectModels.end(); ++it)
                {
                    ROS_INFO("\tid: %d",(*it)->id);
                    ROS_INFO("\thoushold database id: %d",(*it)->dbId);
                }

                bool newModels=false;

                get_model_list_srv_.request.model_set = MODEL_SET;
                if (get_model_list_srv_client_.call(get_model_list_srv_))
                {
                    ROS_INFO("New model sets acquisition and training:");
                    for(size_t i=0; i< get_model_list_srv_.response.model_ids.size(); ++i)
                    {
                        bool newModel=true;
                        for(it=modelsData->objectModels.begin(); it< modelsData->objectModels.end(); ++it)
                        {
                            if((*it)->dbId==get_model_list_srv_.response.model_ids[i])
                                newModel=false;
                        }
                        // Model already trained
                        if(!newModel)
                            continue;
                        newModels=true;

                        // Register model id
                        modelsData->newModel(get_model_list_srv_.response.model_ids[i],loadModelCloud(get_model_list_srv_.response.model_ids[i], scale, n_));

                    }

                    if(!newModels)
                        ROS_INFO("\tNo new models");
                    ROS_INFO("Done");

                }
                else
                {
                    ROS_ERROR("Failed to call service %s",GET_MODEL_LIST_SRV);
                    return false;
                }

                if(newModels)
                {
                    ROS_INFO("Writing data to output file...");
                    modelsData->saveData(modelsFile.c_str());
                    ROS_INFO("Done");
                }
            }
            else
            {
                ROS_INFO("Models loaded: %d", (int) modelsData->objectModels.size());
                for(it=modelsData->objectModels.begin(); it<modelsData->objectModels.end(); ++it)
                {
                    ROS_INFO("\tid: %d",(*it)->id);
                    ROS_INFO("\tdb id: %d",(*it)->dbId);
                }
            }
            return true;
        }*/

    static bool loadModelsTest(boost::shared_ptr<models<objectModelT> > modelsData, float scale, ros::NodeHandle n_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudDownSampled;
        pcl::PointCloud<pcl::PointNormal>::Ptr modelCloudNormals;

        // Service client for getting the objects list from the database
        ros::ServiceClient get_model_list_srv_client_ = n_.serviceClient<household_objects_database_msgs::GetModelList> (GET_MODEL_LIST_SRV, true);
        // Service specifications
        household_objects_database_msgs::GetModelList get_model_list_srv_;

        while ( !ros::service::waitForService(GET_MODEL_LIST_SRV, ros::Duration(2.0)) && n_.ok() )
        {
            ROS_INFO("Waiting for %s service to come up", GET_MODEL_LIST_SRV);
        }
        if (!n_.ok())
            exit(0);


        typename std::vector<objectModelT>::iterator it;

        // Train mode

        get_model_list_srv_.request.model_set = MODEL_SET;
        if (get_model_list_srv_client_.call(get_model_list_srv_))
        {
            ROS_INFO("New model sets acquisition and training:");
            for(size_t i=0; i< get_model_list_srv_.response.model_ids.size(); ++i)
            {

                // Register model id
                if(!( modelsData->newModel(get_model_list_srv_.response.model_ids[i],loadModelCloud(get_model_list_srv_.response.model_ids[i], scale, n_)) ))
                {
                    ROS_INFO("Model already trained: %d", (int)modelsData->objectModels.size());
                }
            }


            ROS_INFO("Done");
        }
        else
        {
            ROS_ERROR("Failed to call service %s",GET_MODEL_LIST_SRV);
            return false;
        }

        return true;
    }

    // Method for visualizing point clouds in rviz
    template <class PointT>
    static void visualizeInputCloud(typename pcl::PointCloud<PointT>::Ptr pclCloud,ros::NodeHandle n,std::string frame, int color, int markerId, float voxel_scale, float voxel_alpha)
    {
        visualization_msgs::Marker object;

        object.header.frame_id = frame;
        object.header.stamp = ros::Time::now();

        object.ns = "scene_cloud";
        object.id = markerId;

        //object.type = visualization_msgs::Marker::CUBE;
        object.type = visualization_msgs::Marker::POINTS;
        object.action = visualization_msgs::Marker::ADD;

        //object.pose.position.x=0.0;
        //object.pose.position.y=0.0;
        //object.pose.position.z=0.0;
        //object.pose.orientation.x = 0.0;ERROR! Incoherent hypothesis number
        //object.pose.orientation.y = 0.0;
        //object.pose.orientation.z = 0.0;
        //object.pose.orientation.w = 1.0;

        // POINTS markers use x and y scale for width/height respectively
        object.scale.x = voxel_scale;
        object.scale.y = voxel_scale;
        object.scale.z = voxel_scale;

        // Points color
        object.color.a = voxel_alpha;
        if(color==0)
        {
            object.color.r = 1.0f;
            object.color.g = 0.0f;
            object.color.b = 0.0f;
        }
        else if(color==1)
        {
            object.color.r = 0.0f;
            object.color.g = 1.0f;
            object.color.b = 0.0f;
        }
        else
        {
            object.color.r = 0.0f;
            object.color.g = 0.0f;
            object.color.b = 1.0f;
        }

        object.lifetime = ros::Duration();


        // Create the vertices for the points and lines
        for(size_t i=0; i<pclCloud->points.size(); i++)
        {
            geometry_msgs::Point p;

            p.x = pclCloud->points[i].x;
            p.y = pclCloud->points[i].y;
            p.z = pclCloud->points[i].z;
            object.points.push_back(p);
        }

        marker_pub.publish(object);

    }

    template <class PointT>
    static void visualize(std::vector<typename pcl::PointCloud<PointT>::Ptr> pclClouds,ros::NodeHandle & n,std::string frame, int color, int markerId, std::string nameSpace, float voxel_scale, float voxel_alpha)
    {
        visualization_msgs::Marker object;

        //object.header.frame_id =  "/openni_depth_frame";
        object.header.frame_id =  frame;
        object.header.stamp = ros::Time::now();

        object.ns = nameSpace;
        object.id = markerId;

        //object.type = visualization_msgs::Marker::CUBE;
        object.type = visualization_msgs::Marker::POINTS;
        object.action = visualization_msgs::Marker::ADD;

        //object.pose.position.x=0.0;
        //object.pose.position.y=0.0;
        //object.pose.position.z=0.0;
        //object.pose.orientation.x = 0.0;
        //object.pose.orientation.y = 0.0;
        //object.pose.orientation.z = 0.0;
        //object.pose.orientation.w = 1.0;

        // POINTS markers use x and y scale for width/height respectively
        object.scale.x = voxel_scale;
        object.scale.y = voxel_scale;
        object.scale.z = voxel_scale;

        // Points color
        object.color.a = voxel_alpha;
        if(color==0)
        {
            object.color.r = 1.0f;
            object.color.g = 0.0f;
            object.color.b = 0.0f;
        }
        else if(color==1)
        {
            object.color.r = 0.0f;
            object.color.g = 1.0f;
            object.color.b = 0.0f;
        }
        else
        {
            object.color.r = 0.0f;
            object.color.g = 0.0f;
            object.color.b = 1.0f;
        }

        object.lifetime = ros::Duration();

        // For each point cloud
        for(typename std::vector<typename pcl::PointCloud<PointT>::Ptr>::iterator cIt=pclClouds.begin(); cIt<pclClouds.end(); cIt++)
        {
            // Create the vertices for the points and lines
            for(size_t i=0; i<(*cIt)->points.size(); i++)
                //for(pcl::PointCloud<pcl::PointXYZ>::iterator pIt=cIt.begin(); pIt<cIt.end(); pIt++)
            {
                geometry_msgs::Point p;
                //p.x = pIt->x;
                //p.y = pIt->y;
                //p.z = pIt->z;
                p.x = (*cIt)->points[i].x;
                p.y = (*cIt)->points[i].y;
                p.z = (*cIt)->points[i].z;
                object.points.push_back(p);
            }
        }

        marker_pub.publish(object);

    }

    static void visualizeCorrectPoses(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclClouds,ros::NodeHandle n,std::string frame)
    {
        visualization_msgs::Marker object;

        //object.header.frame_id =  "/openni_depth_frame";
        object.header.frame_id =  frame;
        object.header.stamp = ros::Time::now();

        object.ns = "object_recognition";
        object.id = 2;

        //object.type = visualization_msgs::Marker::CUBE;
        object.type = visualization_msgs::Marker::POINTS;
        object.action = visualization_msgs::Marker::ADD;

        //object.pose.position.x=0.0;
        //object.pose.position.y=0.0;
        //object.pose.position.z=0.0;
        //object.pose.orientation.x = 0.0;
        //object.pose.orientation.y = 0.0;
        //object.pose.orientation.z = 0.0;
        //object.pose.orientation.w = 1.0;

        // POINTS markers use x and y scale for width/height respectively
        object.scale.x = 0.01;
        object.scale.y = 0.01;
        object.scale.z = 0.01;


        // Points are green
        object.color.a = 1.0f;
        object.color.r = 0.0f;
        object.color.g = 1.0f;
        object.color.b = 0.0f;


        object.lifetime = ros::Duration();

        // For each point cloud
        for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator cIt=pclClouds.begin(); cIt<pclClouds.end(); cIt++)
        {
            // Create the vertices for the points and lines
            for(size_t i=0; i<(*cIt)->points.size(); i++)
                //for(pcl::PointCloud<pcl::PointXYZ>::iterator pIt=cIt.begin(); pIt<cIt.end(); pIt++)
            {
                geometry_msgs::Point p;
                //p.x = pIt->x;
                //p.y = pIt->y;
                //p.z = pIt->z;
                p.x = (*cIt)->points[i].x;
                p.y = (*cIt)->points[i].y;
                p.z = (*cIt)->points[i].z;
                object.points.push_back(p);
            }
        }

        marker_pub.publish(object);

    }

    static void visualizeWrongPoses(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pclClouds,ros::NodeHandle n,std::string frame)
    {
        visualization_msgs::Marker object;

        //object.header.frame_id =  "/openni_depth_frame";
        object.header.frame_id =  frame;
        object.header.stamp = ros::Time::now();

        object.ns = "object_recognition";
        object.id = 3;

        //object.type = visualization_msgs::Marker::CUBE;
        object.type = visualization_msgs::Marker::POINTS;
        object.action = visualization_msgs::Marker::ADD;

        //object.pose.position.x=0.0;
        //object.pose.position.y=0.0;
        //object.pose.position.z=0.0;
        //object.pose.orientation.x = 0.0;
        //object.pose.orientation.y = 0.0;
        //object.pose.orientation.z = 0.0;
        //object.pose.orientation.w = 1.0;

        // POINTS markers use x and y scale for width/height respectively
        object.scale.x = 0.01;
        object.scale.y = 0.01;
        object.scale.z = 0.01;


        // Points are green
        object.color.a = 1.0f;
        object.color.r = 1.0f;
        object.color.g = 0.0f;
        object.color.b = 0.0f;


        object.lifetime = ros::Duration();

        // For each point cloud
        for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator cIt=pclClouds.begin(); cIt<pclClouds.end(); cIt++)
        {
            // Create the vertices for the points and lines
            for(size_t i=0; i<(*cIt)->points.size(); i++)
                //for(pcl::PointCloud<pcl::PointXYZ>::iterator pIt=cIt.begin(); pIt<cIt.end(); pIt++)
            {
                geometry_msgs::Point p;
                //p.x = pIt->x;
                //p.y = pIt->y;
                //p.z = pIt->z;
                p.x = (*cIt)->points[i].x;
                p.y = (*cIt)->points[i].y;
                p.z = (*cIt)->points[i].z;
                object.points.push_back(p);
            }
        }

        marker_pub.publish(object);
    }

    // convert quaternion to euler
    static void quaternionToEuler(const Eigen::Quaternion<float> & q, float & roll, float & pitch, float & yaw)
    {
        float test = q.x()*q.z() + q.y()*q.w();

        if (test > 0.49999)
        { // singularity at north pole
            //std::cout << "OLA" << std::endl;

            roll = 0.0; // symmetric!!
            //roll = 2 * atan2(q.z(),q.w());
            pitch = PI/2.0;
            yaw = 0.0;
            return;
        }
        if (test < -0.49999)
        { // singularity at south pole
            //std::cout << "DEUS" << std::endl;

            roll = 0.0; // symmetric!!
            //roll = -2 * atan2(q.z(),q.w());
            pitch = - PI/2.0;
            yaw = 0.0;
            return;
        }
        float sqx = q.x()*q.x();
        float sqy = q.y()*q.y();
        //float sqz = q.z()*q.z();

        roll = atan2f(2*q.x()*q.w()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqy);
        pitch = asinf(2*test);
        //yaw = atan2f(2*q.z()*q.w()-2*q.y()*q.x() , 1 - 2*sqz - 2*sqy);
        yaw=0.0; // symmetric!!

    }


    // convert euler angles to quaternion
    static void eulerToQuaternion(Eigen::Quaternion<float> & q, const float & roll, const float & pitch, const float & yaw)
    {
        float roll2=roll*0.5;
        float pitch2=pitch*0.5;
        float yaw2=yaw*0.5;
        // Assuming the angles are in radians.
        float c1 = cos(roll2);
        float s1 = sin(roll2);
        float c2 = cos(pitch2);
        float s2 = sin(pitch2);
        float c3 = cos(yaw2);
        float s3 = sin(yaw2);
        float c1c2 = c1*c2;
        float s1s2 = s1*s2;
        q.w() =c1c2*c3 - s1s2*s3;
        q.x() =s1*c2*c3 + c1*s2*s3;
        q.y() =c1*s2*c3 - s1*c2*s3;
        q.z() =c1c2*s3 + s1s2*c3;

        return;
    }

    // convert euler angles to an index
    static int eulerToIndex(float & roll, float & pitch, float & yaw, int angleBins)
    {
        int rollDiscrete=floor(((roll+PI)/(2*PI)) * angleBins); // roll [-pi;pi]
        if(rollDiscrete==angleBins) --rollDiscrete;
        int pitchDiscrete=floor(((pitch+PI*0.5)/(2*PI)) * angleBins); // pitch [-pi/2; pi/2]
        if(pitchDiscrete==angleBins) --pitchDiscrete;
        int yawDiscrete=floor(((yaw+PI)/(2*PI)) * angleBins); // yaw [-pi;pi]
        if(yawDiscrete==angleBins) --yawDiscrete;
        //std::cout << yaw << std::endl;
        int index=rollDiscrete + (pitchDiscrete*angleBins) + (yawDiscrete*angleBins*angleBins);
        if(index>(angleBins*angleBins*angleBins))
        {

            std::cout << "ups:" << std::endl;
            std::cout << "angle bins:" << angleBins << std::endl;
            std::cout << "roll:" << roll*RAD_TO_DEG << " " << rollDiscrete << std::endl;
            std::cout << "pitch:" << pitch*RAD_TO_DEG << " " <<  pitchDiscrete << std::endl;
            std::cout << "yaw:" << yaw*RAD_TO_DEG << " " << yawDiscrete << std::endl;
            std::cout << "adeus:" << std::endl;
            exit(0);
        }

        return index;
        //return ((roll+PI)/(2*PI)) * angleBins + ((pitch+(PI/2))/(PI)) * angleBins*angleBins  + ((yaw+PI)/(2*PI)) *angleBins*angleBins*(0.5*angleBins+1);
    }

    // convert pose to an index
    static int poseToIndex(float & roll, float & pitch, float & yaw, int angleBins, float & x, float & y, float & z, int  positionXBins, int positionYBins, int positionZBins, float & positionXStep, float & positionYStep, float & positionZStep, float positionXOffset, float positionYOffset, float positionZOffset)
    {
        float xCenter=0.4; //martelado para bater certo com as deteccoes que tem uma translacao de 40 cm em x
        // orientation
        int rollDiscrete=floor(((roll+PI)/(2*PI)) * angleBins); // roll [-pi;pi]
        if(rollDiscrete==angleBins) --rollDiscrete;

        int pitchDiscrete=floor(((pitch+PI*0.5)/(2*PI)) * angleBins); // pitch [-pi/2; pi/2]
        if(pitchDiscrete==angleBins) --pitchDiscrete;

        int yawDiscrete=floor(((yaw+PI)/(2*PI)) * angleBins); // yaw [-pi;pi]
        if(yawDiscrete==angleBins) --yawDiscrete;


        unsigned long long xDiscrete=floor((fabs((x-xCenter)+positionXOffset))/positionXStep);
        if(xDiscrete==positionXBins) --xDiscrete;

        unsigned long long yDiscrete=floor((fabs(y+positionYOffset))/positionYStep);
        if(yDiscrete==positionYBins) --yDiscrete;

        unsigned long long zDiscrete=floor((fabs(z+positionZOffset))/positionZStep);
        if(zDiscrete==positionZBins) --zDiscrete;


        unsigned long long index=(unsigned long long)((rollDiscrete) +
                                                      (pitchDiscrete *static_cast<unsigned long long>(angleBins)) +
                                                      (yawDiscrete   *static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)) +
                                                      (xDiscrete     *static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)) +
                                                      (yDiscrete     *static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(positionXBins)) +
                                                      (zDiscrete     *static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(positionXBins)*static_cast<unsigned long long>(positionYBins)));

        unsigned long long maxIndex=static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(angleBins)*static_cast<unsigned long long>(positionXBins)*static_cast<unsigned long long>(positionYBins)*static_cast<unsigned long long>(positionZBins);

        //std::cout << index << " " << maxIndex << std::endl;
        if(index > maxIndex)
        {

            std::cout << "ups:" << std::endl;
            std::cout << "angle bins:" << angleBins << std::endl;
            std::cout << "roll:" << roll*RAD_TO_DEG << " " << rollDiscrete << std::endl;
            std::cout << "pitch:" << pitch*RAD_TO_DEG << " " <<  pitchDiscrete << std::endl;
            std::cout << "yaw:" << yaw*RAD_TO_DEG << " " << yawDiscrete << std::endl << std::endl;
            std::cout << "position x bins:" << positionXBins << std::endl;
            std::cout << "position y bins:" << positionYBins << std::endl;
            std::cout << "position z bins:" << positionZBins << std::endl;
            std::cout << "x:" << x << " " << xDiscrete << std::endl;
            std::cout << "y:" << y << " " << yDiscrete << std::endl;
            std::cout << "z:" << z << " " << zDiscrete << std::endl;
            std::cout << "index: " << index << " max index: " << maxIndex << std::endl;
            exit(0);
        }

        return index;
    }

private:
    // methods
    static pcl::PointCloud<pcl::PointXYZ>::Ptr loadModelCloud(int modelId, float scale, ros::NodeHandle n_)
    {
        ROS_INFO(" Getting model point cloud for the object %d...",modelId);
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud (new pcl::PointCloud<pcl::PointXYZ>);

        // Service client for getting the objects mesh's from the database
        ros::ServiceClient get_model_mesh_srv_client_ = n_.serviceClient<household_objects_database_msgs::GetModelMesh> (GET_MODEL_MESH_SRV, true);
        // Service specifications
        household_objects_database_msgs::GetModelMesh get_model_mesh_srv_;

        while ( !ros::service::waitForService(GET_MODEL_MESH_SRV, ros::Duration(2.0)) && n_.ok() )
        {
            ROS_INFO("Waiting for %s service to come up", GET_MODEL_MESH_SRV);
        }
        if (!n_.ok())
            exit(0);

        get_model_mesh_srv_.request.model_id = modelId;

        if (get_model_mesh_srv_client_.call(get_model_mesh_srv_))
        {
            // GET EACH POINT OF THE POINT CLOUD
            BOOST_FOREACH (const geometry_msgs::Point& pt, get_model_mesh_srv_.response.mesh.vertices)
            {
                modelCloud->push_back(pcl::PointXYZ(pt.x,pt.y,pt.z));
            }

            ROS_INFO("  Point cloud acquired (number of points=%d)", (int)modelCloud->size());
        }
        else
        {
            ROS_ERROR("Failed to call service %s",GET_MODEL_LIST_SRV);
            modelCloud.reset();
        }

        // scale model cloud
        if(equalFloat(scale,1.0000))
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Transform<float,3,Eigen::Affine> scaleTransform=Eigen::Translation<float,3>(0,0,0)*Eigen::UniformScaling<float> (scale);
            pcl::transformPointCloud(*modelCloud, *pclCloudOut,scaleTransform);
            modelCloud=pclCloudOut;
        }

        return modelCloud;
    }
};

