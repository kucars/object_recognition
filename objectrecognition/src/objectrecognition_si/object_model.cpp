#include "objectrecognition_si/object_model.h"

using namespace pcl;

void objectModelSI::seeHashTableEntropy()
{
  	std::vector<pointPairSI>::iterator sameFeatureIt; // same key on hash table
	int key;
	int numberElements;
	int totalElements=0;
	for(int c=0; c< pointPair::angleBins;++c)
		for(int b=0; b< pointPair::angleBins;++b)
			for(int a=0; a< pointPair::angleBins;++a)
				{
					numberElements=0;
					key=(int)(a+(b*pointPair::angleBins)+(c*pointPair::angleBins*pointPair::angleBins));

					for(sameFeatureIt=hashTable[key].begin(); sameFeatureIt < hashTable[key].end(); ++sameFeatureIt)
					{
						++numberElements;
					}
					//if(numberElements>0)
						//ROS_INFO("HASH TABLE [%d]=%d",key,numberElements);
					totalElements=totalElements+numberElements;
				}

	//ROS_INFO("TOTAL ELEMENTS:%d  HASH TABLE SIZE:%d", totalElements, (int)hashTable.size());
			//exit(-1);
}

boost::shared_ptr < transformation >  objectModelSI::modelToScene(const pcl::PointNormal & pointModel, const Eigen::Affine3f & transformSceneToGlobal, const float alpha, const float scale)
{
	Eigen::Vector3f modelPoint=pointModel.getVector3fMap();
	Eigen::Vector3f modelNormal=pointModel.getNormalVector3fMap ();

	// Get scaled model
	Eigen::UniformScaling<float> modelScale(scale);
	Eigen::Vector3f modelPointScaled=modelScale*modelPoint;

	// Get transformation from model local frame to global frame
   	Eigen::AngleAxisf rotationModelToGlobal(acosf (modelNormal.dot (Eigen::Vector3f::UnitX ())), modelNormal.cross (Eigen::Vector3f::UnitX ()).normalized ());
    Eigen::Affine3f transformModelToGlobal = Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelPointScaled)) * rotationModelToGlobal * modelScale;

	// Get transformation from model local frame to scene local frame
    Eigen::Affine3f completeTransform = transformSceneToGlobal.inverse () * Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX ()) * transformModelToGlobal;


	Eigen::Quaternion<float> rotationQ=Eigen::Quaternion<float>(completeTransform.rotation());

	// if object is symmetric remove yaw rotation (assume symmetry around z axis)
	if(symmetric)
	{
		Eigen::Vector3f eulerAngles;
		// primeiro [0] -> rot. around x (roll) [1] -> rot. around y (pitch) [2] -> rot. around z (yaw)
		//pcl::getEulerAngles(completeTransform,eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		quaternionToEuler(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//eulerAngles[2]=0;
		eulerToQuaternion(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//quaternionToEuler(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//std::cout << "EULER ANGLES: " << eulerAngles << std::endl;
	}

	//boost::shared_ptr < transformation > transf(new transformation(rotationQ,  Eigen::Translation3f(trans[0],trans[1],trans[2])));
	//boost::shared_ptr < transformation > transf(new transformation(rotationQ, Eigen::Translation3f(completeTransform.translation()) ));

	return boost::shared_ptr < transformation > (new transformation(rotationQ, Eigen::Translation3f(completeTransform.translation()),modelScale));
}

void objectModelSI::train()
{
	//static int aux=0;
	// Iterators
	pcl::PointCloud<pcl::PointNormal>::iterator mr;
	pcl::PointCloud<pcl::PointNormal>::iterator mi;

	int idRef=0;

	/*boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer = objectModel::viewportsVis(modelCloud);

  	while (!viewer->wasStopped ())
  	{
   		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/

	std::cout << "  Downsample dense surflet cloud... " << std::endl;
	std::cout << "   Surflet cloud size before downsampling: " << modelCloud->size() << std::endl;
 	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (modelCloud);
  	sor.setLeafSize (distanceStep,distanceStep,distanceStep);
  	sor.filter (*modelCloud);
  	std::cout << "   Surflet cloud size after downsampling: " << modelCloud->size() << std::endl;
	std::cout<< "  Done";

	/*
	boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(modelCloud);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/

	std::cout<< "  Reestimate normals... " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudPoint(new pcl::PointCloud<pcl::PointXYZ>);
	for(mr=modelCloud->begin(); mr<modelCloud->end(); mr++)
	{
		modelCloudPoint->push_back(pcl::PointXYZ(mr->x,mr->y,mr->z));
	}
	computeNormals(modelCloudPoint,modelCloud);
	std::cout<< "  Done " << std::endl;

/*
	boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer3 = objectModel::viewportsVis(modelCloud);

  	while (!viewer3->wasStopped ())
  	{
   		viewer3->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}
*/

	hashTable.resize(pointPairSI::maxHash);
	for(mr=modelCloud->begin(); mr<modelCloud->end(); mr++)
	{
		// Add reference point to point cloud
		for(mi=modelCloud->begin();mi<modelCloud->end(); mi++)
		{
			// If same point... continue...
			if(mr==mi)
			{
				continue;
			}

			// Compute SIPPF
			pointPairSI SIPPF=pointPairSI(*mr,*mi,idRef);

			// Insert PPF on the hashtable
			hashTable[SIPPF.getHash(*mi)].push_back(SIPPF);
		}

		idRef++;
	}		
	std::cout << "  Hash table size: " << hashTable.size() << std::endl;
}

//int objectModelSI::computeIndexDebug(const Eigen::Vector4f& feature)
//{
//	Eigen::Vector3d featureDiscrete;
//
//	// Discretize the output
//	featureDiscrete[0]=lutCosToAngle[ floor( (feature[1]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	featureDiscrete[1]=lutCosToAngle[ floor( (feature[2]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	featureDiscrete[2]=lutCosToAngle[ floor( (feature[3]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	std::cout << "feature discreta:" << featureDiscrete << std::endl;
//	std::cout << " ..." << floor( (feature[2]+1.0) * static_cast<float>(cosResolution)*0.5) << std::endl;
//	std::cout << " ..." << feature[2]+1.0<< std::endl;
//	return 	(featureDiscrete[0]+(featureDiscrete[1]*angleBins)+(featureDiscrete[2]*angleBins*angleBins));
//}



