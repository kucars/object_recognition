#include "objectrecognition_sv/object_model.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/vector_average.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud)
{
	// --------------------------------------------------------
  	// -----Open 3D viewer and add point cloud and normals-----
  	// --------------------------------------------------------
  	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (255, 255, 255);
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
  	viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, rgb, "sample cloud");
 	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud");
  	//viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal> (cloud, normals, 1, 0.01, "normals");
  	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud,1,1.0,"normals");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, static_cast<float>(205.0/255.0), static_cast<float>(201.0/255.0), static_cast<float>(201.0/255.0), "normals");
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals"); 
  	viewer->addCoordinateSystem (0.1);
 	viewer->initCameraParameters ();
  	//viewer->addText("Z", 10, 10, "v1 text", 0);
	pcl::PointXYZ textXPos(0.105,-0.01,0.0);
	pcl::PointXYZ textYPos(0.0,0.105,0.0);
	pcl::PointXYZ textZPos(0.0,0.0,0.105);
	float textScale=0.01;
	viewer->addText3D ("x", textXPos, textScale, 0.5, 0.5, 0.5, "x");
	viewer->addText3D ("y", textYPos, textScale, 0.5, 0.5, 0.5, "y");
	viewer->addText3D ("z", textZPos, textScale, 0.5, 0.5, 0.5, "z");
  	//viewer->setCameraPosition ( 0.5,0.1,0.2, 0, 0, 1);

  	return (viewer);
}

void reAlignPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr _point_cloud)
{
	pcl::VectorAverage<float,3> va;
 	Eigen::Vector3f _position;
 	Eigen::Vector3f _eigen_values; 
 	std::vector<Eigen::Vector3f> _eigen_vector;
	_eigen_vector.resize(3);
 	Eigen::Vector3f _bounding_box; 
	Eigen::Transform<float, 3, Eigen::Affine> _pose; 
	Eigen::Vector4f min_pt;
	Eigen::Vector4f max_pt;

	/////////////////
	// Compute PCA //
	/////////////////

	for(unsigned int i=0; i<_point_cloud->points.size(); ++i)
	{
		va.add(Eigen::Matrix<float, 3, 1> (_point_cloud->points[i].x, _point_cloud->points[i].y, _point_cloud->points[i].z));
	}

	va.doPCA(_eigen_values, _eigen_vector[0], _eigen_vector[1], _eigen_vector[2]);

	_position = va.getMean();

	_position[2]=0.0;
	va.reset();

	_eigen_values[0]=sqrt(_eigen_values[0]);
	_eigen_values[1]=sqrt(_eigen_values[1]);
	_eigen_values[2]=sqrt(_eigen_values[2]);
	float d1=_eigen_values[2];
	float d2=_eigen_values[1];
	float d3=_eigen_values[0];
	std::cout << "EIGEN_VALUES:" << _eigen_values << std::endl;

	//Eigen::Matrix3f rotation_matrix;
	//rotation_matrix << _eigen_vector[0], _eigen_vector[1], _eigen_vector[2];
	Eigen::AngleAxisf angleAxis;
	//angleAxis = Eigen::Matrix3f(3,3);
	//angleAxis = rotation_matrix;

	Eigen::Vector3f best_vector;
	float best_vector_dot=0.0;
	for(int i=0; i<3; ++i)
	{
		float temp_dot=_eigen_vector[i].dot(Eigen::Vector3f::UnitZ());
		std::cout << "temp_dot:" << temp_dot << std::endl;
		if(fabs(temp_dot)>best_vector_dot)
		{

			best_vector_dot=fabs(temp_dot);
			if(temp_dot>0.f)
				best_vector=_eigen_vector[i];
			else
				best_vector=-_eigen_vector[i];
			std::cout << "best_vector:" << best_vector << std::endl;
		}
	}
	best_vector.normalize();
	angleAxis=Eigen::AngleAxisf(-acos(best_vector.dot(Eigen::Vector3f::UnitZ())), best_vector.cross(Eigen::Vector3f::UnitZ()).normalized());
	std::cout << "rot:" << 180*acos(best_vector.dot(Eigen::Vector3f::UnitZ()))/PI << std::endl;
	
	std::cout << "pos:" << _position << std::endl;
	_pose=Eigen::Translation3f(_position)*angleAxis;
	pcl::PointCloud<pcl::PointNormal> centered_point_cloud;
	pcl::transformPointCloudWithNormals(*_point_cloud, centered_point_cloud, (Eigen::Affine3f) _pose.inverse());





	for(unsigned int i=0; i<centered_point_cloud.points.size(); ++i)
	{
		va.add(Eigen::Matrix<float, 3, 1> (centered_point_cloud.points[i].x, centered_point_cloud.points[i].y, centered_point_cloud.points[i].z));
	}

	va.doPCA(_eigen_values, _eigen_vector[0], _eigen_vector[1], _eigen_vector[2]);

	_position = va.getMean();

	_position[2]=0.0;
	va.reset();

	_eigen_values[0]=sqrt(_eigen_values[0]);
	_eigen_values[1]=sqrt(_eigen_values[1]);
	_eigen_values[2]=sqrt(_eigen_values[2]);
	std::cout << "EIGEN_VALUES:" << _eigen_values << std::endl;

	//Eigen::Matrix3f rotation_matrix;
	//rotation_matrix << _eigen_vector[0], _eigen_vector[1], _eigen_vector[2];

	//angleAxis = Eigen::Matrix3f(3,3);
	//angleAxis = rotation_matrix;



	for(int i=0; i<3; ++i)
	{
		float temp_dot=_eigen_vector[i].dot(Eigen::Vector3f::UnitZ());
		std::cout << "temp_dot:" << temp_dot << std::endl;
		if(fabs(temp_dot)>best_vector_dot)
		{

			best_vector_dot=fabs(temp_dot);
			if(temp_dot>0.f)
				best_vector=_eigen_vector[i];
			else
				best_vector=-_eigen_vector[i];
			std::cout << "best_vector:" << best_vector << std::endl;
		}
	}


	////////////////////////
	// Recompute position //
	////////////////////////
*_point_cloud=centered_point_cloud;
	/*pcl::getMinMax3D(centered_point_cloud, min_pt, max_pt);

	Eigen::Vector3f position_temp;
	position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=0.0;
	std::cout << "position temp before:" << position_temp << std::endl;
	_position=_position+( _pose.rotation() * position_temp );
	_pose=Eigen::Translation3f(_position)*angleAxis;
	pcl::transformPointCloudWithNormals(*_point_cloud, centered_point_cloud, (Eigen::Affine3f) _pose.inverse());
	*_point_cloud=centered_point_cloud;

	pcl::getMinMax3D(centered_point_cloud, min_pt, max_pt);
	position_temp[0]=( max_pt.x() + min_pt.x() )/2.00000;
	position_temp[1]=( max_pt.y() + min_pt.y() )/2.00000;
	position_temp[2]=0.0;
	std::cout << "position temp after:" << position_temp << std::endl;*/
}

void addPointRGB(pcl::PointNormal & mp, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudRGB, int type)
{
	pcl::PointXYZRGBNormal pointRGB;
	//pcl::Normal normal;
	int8_t r(0),g(0),b(0);
	if(type==1)
	{
		std::cout << "type 1" << std::endl;
		r=255; g=0; b=0;
	}
	else if(type==2)
	{
		std::cout << "type 2" << std::endl;
		//r=2; g=200; b=2;
		r=0; g=255; b=0;	
	}

	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	pointRGB.rgb = *reinterpret_cast<float*>(&rgb);

	pointRGB.x=mp.x;
	pointRGB.y=mp.y;
	pointRGB.z=mp.z;

	pointRGB.normal_x=mp.normal_x;
	pointRGB.normal_y=mp.normal_y;
	pointRGB.normal_z=mp.normal_z;

	pointCloudRGB->push_back(pointRGB);
	// add normals
	/*normal.normal_x=mp.normal_x;
	normal.normal_y=mp.normal_y;
	normal.normal_z=mp.normal_z;

	normals->push_back(normal);*/
}


using namespace pcl;

void objectModelSV::seeHashTableEntropy()
{
	int key;
	int numberElements;
	int totalVotes=0;
	int totalElements=0;
	for(std::vector<std::vector<pointPairSV> >::iterator hashIt=hashTable.begin(); hashIt < hashTable.end(); ++hashIt)
	{
		numberElements=0;

		for(std::vector<pointPairSV>::iterator sameFeatureIt=hashIt->begin(); sameFeatureIt < hashIt->end(); ++sameFeatureIt)
		{
			totalVotes+=sameFeatureIt->weight;
			++numberElements;
		}
		//if(numberElements>0)
		//ROS_INFO("HASH TABLE [%d]=%d",key,numberElements);
		totalElements=totalElements+numberElements;
	}

	ROS_INFO("TOTAL ELEMENTS:%d  HASH TABLE SIZE:%d  TOTAL VOTES:%d", totalElements, (int)hashTable.size(), totalVotes);
	exit(-1);
}


transformation objectModelSV::modelToScene(const pcl::PointNormal & pointModel, const Eigen::Affine3f & transformSceneToGlobal, const float alpha)
{
	Eigen::Vector3f modelPoint=pointModel.getVector3fMap();
	Eigen::Vector3f modelNormal=pointModel.getNormalVector3fMap ();

	// Get transformation from model local frame to global frame
	Eigen::Vector3f cross=modelNormal.cross (Eigen::Vector3f::UnitX ()).normalized ();
	Eigen::AngleAxisf rotationModelToGlobal(acosf (modelNormal.dot (Eigen::Vector3f::UnitX ())), cross);

	if (isnan(cross[0]))
	{
		rotationModelToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
	}		
	//std::cout<< "ola:" <<acosf (modelNormal.dot (Eigen::Vector3f::UnitX ()))<<std::endl;
	//std::cout <<"X:"<< Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelPoint)).x() << std::endl;
	//std::cout <<"Y:"<< Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelPoint)).y() << std::endl;
	//std::cout <<"Z:"<< Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelPoint)).z() << std::endl;

    Eigen::Affine3f transformModelToGlobal = Eigen::Translation3f( rotationModelToGlobal * ((-1) * modelPoint)) * rotationModelToGlobal;

	// Get transformation from model local frame to scene local frame
    Eigen::Affine3f completeTransform = transformSceneToGlobal.inverse () * Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX ()) * transformModelToGlobal;

	//std::cout << Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX ()).matrix() << std::endl;

	Eigen::Quaternion<float> rotationQ=Eigen::Quaternion<float>(completeTransform.rotation());

	// if object is symmetric remove yaw rotation (assume symmetry around z axis)
	if(symmetric)
	{
		Eigen::Vector3f eulerAngles;
		// primeiro [0] -> rot. around x (roll) [1] -> rot. around y (pitch) [2] -> rot. around z (yaw)
		quaternionToEuler(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//pcl::getEulerAngles(completeTransform,eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//eulerAngles[2]=0.0;
		eulerToQuaternion(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//quaternionToEuler(rotationQ, eulerAngles[2], eulerAngles[1], eulerAngles[2]);
		//std::cout << "EULER ANGLES: " << eulerAngles << std::endl;
	}


	//std::cout << "rotation: " << rotationQ << std::endl;
	return transformation(rotationQ, Eigen::Translation3f(completeTransform.translation()) );
}


transformation  objectModelSV::modelToScene( const int modelPointIndex, const Eigen::Affine3f & transformSceneToGlobal, const float alpha)
{
	Eigen::Vector3f modelPoint=modelCloud->points[modelPointIndex].getVector3fMap();
	Eigen::Vector3f modelNormal=modelCloud->points[modelPointIndex].getNormalVector3fMap ();

	// Get transformation from model local frame to scene local frame
    Eigen::Affine3f completeTransform = transformSceneToGlobal.inverse () * Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX ()) * modelCloudTransformations[modelPointIndex];

	Eigen::Quaternion<float> rotationQ=Eigen::Quaternion<float>(completeTransform.rotation());

	// if object is symmetric remove yaw rotation (assume symmetry around z axis)
	if(symmetric)
	{
		Eigen::Vector3f eulerAngles;
		// primeiro [0] -> rot. around x (roll) [1] -> rot. around y (pitch) [2] -> rot. around z (yaw)
		quaternionToEuler(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//pcl::getEulerAngles(completeTransform,eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//eulerAngles[2]=0.0;
		eulerToQuaternion(rotationQ, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		//quaternionToEuler(rotationQ, eulerAngles[2], eulerAngles[1], eulerAngles[2]);
		//std::cout << "EULER ANGLES: " << eulerAngles << std::endl;
	}
	//std::cout << "translation: " << completeTransform.rotation().matrix() << std::endl;
	return transformation(rotationQ, Eigen::Translation3f(completeTransform.translation()) );
}





void objectModelSV::train()
{
	// Iterators
	pcl::PointCloud<pcl::PointNormal>::iterator mr;
	pcl::PointCloud<pcl::PointNormal>::iterator mi;

	int idRef=0;

	// TRUE FOR CYLLINDER
	/*for(int i=0; i<modelCloud->size(); ++i)
	{

		if(modelCloud->points[i].normal_z>0.001 || modelCloud->points[i].normal_z<-0.001) // Cyllinder stuff
			modelCloud->points[i].normal_z=0.0;


		double inner =  modelCloud->points[i].normal_x*modelCloud->points[i].x + 
				modelCloud->points[i].normal_y*modelCloud->points[i].y;
		if(inner<0)
		{
			modelCloud->points[i].normal_x=-modelCloud->points[i].normal_x;
			modelCloud->points[i].normal_y=-modelCloud->points[i].normal_y;
		}

		modelCloud->points[i].normal_z=0.0;

	}*/
	// END TRUE FOR CYLLINDER

	std::cout << "  Downsample dense surflet cloud... " << std::endl;
	std::cout << "   Surflet cloud size before downsampling: " << modelCloud->size() << std::endl;
 	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (modelCloud);
  	//std::cout << "subsampleStep: " << subsampleStep <<" distanceStep: "<<  distanceStep << std::endl;
  	//sor.setLeafSize (subsampleStep,subsampleStep,subsampleStep);
  	sor.setLeafSize (distanceStep,distanceStep,distanceStep);
  	sor.filter (*modelCloud);
	std::cout << "   Surflet cloud size after downsampling: " << modelCloud->size() << std::endl;
	std::cout << "  Done" << std::endl;

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(modelCloud);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/

	std::cout<< "  Re-estimate normals... " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudPoint(new pcl::PointCloud<pcl::PointXYZ>);
	for(mr=modelCloud->begin(); mr<modelCloud->end(); mr++)
	{
		modelCloudPoint->push_back(pcl::PointXYZ(mr->x,mr->y,mr->z));
	}
	computeNormals(modelCloudPoint,modelCloud);
	std::cout << "  Done" << std::endl;

    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(modelCloud);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/



	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////
	pcl::PointIndices normals_nan_indices;

	for (unsigned int i = 0; i < modelCloud->points.size(); ++i) 
	{
		if (isnan(modelCloud->points[i].normal[0]) || isnan(modelCloud->points[i].normal[1]) || isnan(modelCloud->points[i].normal[2]))
		{
	   		normals_nan_indices.indices.push_back(i);
		}
	}



	pcl::ExtractIndices<pcl::PointNormal> nan_extract;
	nan_extract.setInputCloud(modelCloud);
	nan_extract.setIndices(boost::make_shared<pcl::PointIndices> (normals_nan_indices));
	nan_extract.setNegative(true);
	nan_extract.filter(*modelCloud);
	std::cout<< "\tCloud size after removing NaN normals: " << modelCloud->size() << endl;

	hashTable.resize(pointPairSV::maxHash);
	int features_perdidas_por_erro_de_amostragem=0;
	if(symmetric)	
	{

		
		//reAlignPointCloud(modelCloud);

		float discardedPointPairs=0;
		float remainingPointPairs=0;

			////////////////////////////
			// COMPUTE MODEL DIAMETER //
			////////////////////////////

			float maxSquaredDistance=0.0;
			float minSquaredDistance=10000.0;
			int ref_max=0;
			int sec_min=1;
			int r=0;

			Eigen::Vector3f max_dist_vector;

			for(mr=modelCloud->begin(); mr<modelCloud->end(); ++mr)
			{
				int i=0;
				for(mi=modelCloud->begin();mi<modelCloud->end(); ++mi)
				{
					if(mr==mi)
					{
						continue;
					}
					float squaredDistance=(mr->x-mi->x)*(mr->x-mi->x)+(mr->y-mi->y)*(mr->y-mi->y)+(mr->z-mi->z)*(mr->z-mi->z);
					if(squaredDistance>maxSquaredDistance)
					{
						maxSquaredDistance=squaredDistance;
						ref_max=r;
						sec_min=i;
						max_dist_vector=Eigen::Vector3f(modelCloud->points[ref_max].x-modelCloud->points[sec_min].x,modelCloud->points[ref_max].y-modelCloud->points[sec_min].y,modelCloud->points[ref_max].z-modelCloud->points[sec_min].z);

					}
					if(squaredDistance<minSquaredDistance)
					{
						minSquaredDistance=squaredDistance;
					}
					++i;
				}
				++r;
			}
			//std::cout << modelCloud->points[ref_max] << std::endl;
			//std::cout << ref_max << " " << sec_min << std::endl;
			//std::cout << maxSquaredDistance << std::endl;
			float minDistance=sqrt(minSquaredDistance);
			//std::cout << "min dist:" <<minDistance << std::endl;
			// look for second furthest point
			Eigen::Vector3f second_max_dist_vector;
			maxSquaredDistance=0.0;
			int ola;
			for(int pi=0; pi<modelCloud->points.size(); ++pi)
			{
				if(pi==ref_max||pi==sec_min)
				{
						continue;
				}
				float squaredDistance=(modelCloud->points[ref_max].x-modelCloud->points[pi].x)*(modelCloud->points[ref_max].x-modelCloud->points[pi].x)+(modelCloud->points[ref_max].y-modelCloud->points[pi].y)*(modelCloud->points[ref_max].y-modelCloud->points[pi].y)+(modelCloud->points[ref_max].z-modelCloud->points[pi].z)*(modelCloud->points[ref_max].z-modelCloud->points[pi].z);
				if(squaredDistance>maxSquaredDistance)
				{
						ola=pi;
						maxSquaredDistance=squaredDistance;
						second_max_dist_vector=Eigen::Vector3f(modelCloud->points[ref_max].x-modelCloud->points[pi].x,modelCloud->points[ref_max].y-modelCloud->points[pi].y,modelCloud->points[ref_max].z-modelCloud->points[pi].z);
				}
			}
			//std::cout << modelCloud->points[ref_max] << std::endl;	
			//std::cout << modelCloud->points[sec_min] << std::endl;
			//std::cout << modelCloud->points[ola] << std::endl;
			//std::cout << max_dist_vector << std::endl;
			//std::cout << second_max_dist_vector << std::endl;
			//std::cout << ref_max << " " << ola << std::endl;
			//std::cout << maxSquaredDistance << std::endl;
			//std::cout << "norm before: " << max_dist_vector.norm() << " " << second_max_dist_vector.norm() << std::endl;
			second_max_dist_vector.normalize();
			max_dist_vector.normalize();
			//std::cout << "norm after: " << max_dist_vector.norm() << " " << second_max_dist_vector.norm() << std::endl;
			float angle_threshold=acos(max_dist_vector.dot(second_max_dist_vector));
		
			//std::cout << "angle threshold: " << 180*angle_threshold/(2*PI) << " " << 180*pointPair::angleStep/(2*PI)<< std::endl;
			//std::cout << "distance threshold: " << minDistance << " " <<  distanceStep << std::endl;
			int r_num=0;
		for(mr=modelCloud->begin(); mr<modelCloud->end(); ++mr)
		{
			float discardedNow=0;
			// Add reference point to point cloud
			r_num++;


			Eigen::Vector3f newPoint=mr->getVector3fMap();
			Eigen::Vector3f newNormal=mr->getNormalVector3fMap ();
			Eigen::Vector3f cross=newNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

			Eigen::AngleAxisf rotationToGlobal;
			if(newNormal.isApprox(Eigen::Vector3f::UnitX (),0.00001))
			{
				rotationToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
			}
			else
			{
				cross=newNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

				if (isnan(cross[0]))
				{
					std::cout << "YA"<< std::endl;
					exit(-1);
					rotationToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
				}
				else
				{
					rotationToGlobal=Eigen::AngleAxisf(acosf (newNormal.dot (Eigen::Vector3f::UnitX ())),cross);
				}
			}

			Eigen::Affine3f transformToGlobal = Eigen::Translation3f ( rotationToGlobal* ((-1)*newPoint)) * rotationToGlobal;
			//int mi_num=0;
			for(mi=modelCloud->begin();mi<modelCloud->end(); ++mi)
			{
				// If same point... continue...
				if(mr==mi)
				{
					continue;
				}
				float dist=sqrt((mr->x-mi->x)*
					  	  (mr->x-mi->x)+
					  	  (mr->y-mi->y)*
					  	  (mr->y-mi->y)+
					  	  (mr->z-mi->z)*
					          (mr->z-mi->z));

				// Compute PPF
				pointPairSV PPF=pointPairSV(*mr,*mi, transformToGlobal, idRef);

				// Compute index
				int index=PPF.getHash(*mi,distanceStepInverted);

				if(index >=(int)hashTable.size())
				{
					std::cout << "bolas " << std::endl;

				}


				// Get transformation from scene frame to global frame

				//float alpha=getRotation(*mr,*mi);
				bool newPair=true;
				int pointPairIndex=0;
				//bool match=false;
				//float temp_translation;
				//float temp_rot;
				//float temp_alpha;
				//int match_id;
				//int id=0;
				//std::cout << "mi_num:" << mi_num++ << std::endl;
				for(std::vector<pointPairSV>::iterator sameFeatureIt=hashTable[index].begin(); sameFeatureIt<hashTable[index].end(); ++sameFeatureIt)
				{	
					//match=true;
					//match_id=id++;
					//std::cout << "match" << std::endl;
					float alpha=sameFeatureIt->alpha-PPF.alpha; // alpha values between [-360,360]
					//temp_alpha=alpha;
					//std::cout <<"alpha:"<<sameFeatureIt->alpha << " " << PPF.alpha << std::endl;
					// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
					if(alpha<(-PI))
						alpha=ANGLE_MAX+alpha;
					else if(alpha>(PI))
						alpha=alpha-ANGLE_MAX;

					transformation transf = modelToScene(sameFeatureIt->referencePoint, transformToGlobal, alpha);
					
					Eigen::Vector3f translation(transf.translation.x(),transf.translation.y(),transf.translation.z());
					//temp_translation=translation.norm();
					//std::cout << "temp_trans:"<< temp_translation<< std::endl;
					//if(translation.norm()<(0.95*minDistance))
					//if(translation.norm()<(0.95*minDistance))
					if(translation.norm()<distanceStep)
					//if(translation.norm()<distanceStep)
					{


						Eigen::Quaternion<float> q=transf.rotation;
						q.normalize();
						float roll, pitch, yaw;
						/*float s =sqrt(1.0-q.w()*q.w());
						if(s<0.00001) // angle is close to 0
						{
							newPair=false;
							break;
						}*/
						quaternionToEuler(q, roll, pitch, yaw);
						//yaw=0.0;
						eulerToQuaternion(q, roll, pitch, yaw);
						//Eigen::Vector3f rot_ax(q.x()/s, q.y()/s, q.z()/s);
						//std::cout << "rot_ax:" << rot_ax << std::endl;
						//if(2*acos(rot_ax.dot(Eigen::Vector3f::UnitZ()))<(0.95*angle_threshold))
						//temp_rot=2*acos( fabs( q.w() ));
						if(2*acos( fabs( q.w() ))<pointPair::angleStep)
						//if(2*acos(rot_ax.dot(Eigen::Vector3f::UnitZ()))<pointPair::angleStep/2.0)
						//if(2*acos( fabs( q.w() ))<(0.95*angle_threshold))
						//if(2*acos( fabs( q.w() ))<(0.95*angle_threshold))
						//if(2*acos( fabs( q.w() ))<0.00001)
						//if(fabs(roll)<0.00001 && fabs(pitch)<0.00001)
						{
							//std::cout <<"cool" << std::endl;
							//std::cout <<"translation:" <<translation.norm() << "roll:" << roll << " pitch:" << pitch << " threshold:" <<0.95*angle_threshold <<std::endl;
							newPair=false;
							break;
						}
					}
					//std::cout <<"not cool" << std::endl;

					++pointPairIndex;
				}
				if(newPair)
				{
					/*if(r_num==4)
					{
						//std::cout << PPF<< std::endl;
//						std:cout << "match:" << match << std::endl;
						//std::cout << temp_translation << " " << distanceStep << std::endl;
						//std::cout << temp_rot << " " << pointPair::angleStep << std::endl;
//						std::cout << "alpha:"<< temp_alpha << " " << cos(temp_alpha) <<" " <<cos(-PI) << std::endl;
//						std::cout << "normal ref:" << hashTable[index][match_id].referencePoint.getNormalVector3fMap() << std::endl;
//						std::cout << "normal:" << hashTable[index][match_id].otherPoint.getNormalVector3fMap() << std::endl;

						Eigen::Vector3f teste=hashTable[index][match_id].referencePoint.getNormalVector3fMap();
						teste.normalize();
						std::cout << "normal ref:" << teste << std::endl;

						Eigen::Vector3f cross=teste.cross (Eigen::Vector3f::UnitX ()). normalized();

						//OPTIMIZAR

						Eigen::AngleAxisf rotationToGlobal;

						pcl::PointCloud<pcl::PointNormal>::Ptr pointPairsCloud(new pcl::PointCloud<pcl::PointNormal>);
												pointPairsCloud->push_back(*mr);
												pointPairsCloud->push_back(*mi);
												pointPairsCloud->push_back(hashTable[index][pointPairIndex].referencePoint);
												pointPairsCloud->push_back(hashTable[index][pointPairIndex].otherPoint);
												std::cout << "DISTANCE 1: " << sqrt((mr->x-mi->x)*(mr->x-mi->x) + (mr->y-mi->y)*(mr->y-mi->y)+(mr->z-mi->z)*(mr->z-mi->z)) << std::endl;
												std::cout << "DISTANCE 2: " << sqrt(
						(hashTable[index][pointPairIndex].referencePoint.x-hashTable[index][pointPairIndex].otherPoint.x)*
						(hashTable[index][pointPairIndex].referencePoint.x-hashTable[index][pointPairIndex].otherPoint.x)+
						(hashTable[index][pointPairIndex].referencePoint.y-hashTable[index][pointPairIndex].otherPoint.y)*
						(hashTable[index][pointPairIndex].referencePoint.y-hashTable[index][pointPairIndex].otherPoint.y)+
						(hashTable[index][pointPairIndex].referencePoint.z-hashTable[index][pointPairIndex].otherPoint.z)*
						(hashTable[index][pointPairIndex].referencePoint.z-hashTable[index][pointPairIndex].otherPoint.z)) << std::endl;
												pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudRGB(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
												pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
												for(pcl::PointCloud<pcl::PointNormal>::iterator mp=modelCloud->begin(); mp<modelCloud->end(); ++mp)
												{
													if(mp==mr||mp==mi)
														continue;
													if(mp->x==hashTable[index][pointPairIndex].referencePoint.x && mp->y==hashTable[index][pointPairIndex].referencePoint.y && mp->z==hashTable[index][pointPairIndex].referencePoint.z)
														continue;
													if(mp->x==hashTable[index][pointPairIndex].otherPoint.x && mp->y==hashTable[index][pointPairIndex].otherPoint.y && mp->z==hashTable[index][pointPairIndex].otherPoint.z)
														continue;

													//addPointRGB(*mp, pointCloudRGB, 0);
													addPointRGB(*mp, pointCloudRGB, 0);
												}
												//REF POINT
												addPointRGB(*mr, pointCloudRGB, 2);
												addPointRGB(*mi, pointCloudRGB, 1);

												//addPointRGB(hashTable[index][pointPairIndex].referencePoint, pointCloudRGB, 2);
												addPointRGB(hashTable[index][match_id].otherPoint, pointCloudRGB, 2);

												boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(pointCloudRGB);
												//viewer->addSphere (*mr, distanceStep/2, 1.0, 0.0, 0.0, "sphere_ref_new");
												viewer->addSphere (hashTable[index][match_id].referencePoint, distanceStep/2, 0.0, 1.0, 0.0, "sphere_ref_model");
												while (!viewer->wasStopped ())
												{
													viewer->spinOnce (100);
													boost::this_thread::sleep (boost::posix_time::microseconds (100000));
												}


						exit(-1);
					}*/
					++remainingPointPairs;
					// Insert PPF on the hashtable
					//std::cout << PPF.getHash(*mi,distanceStepInverted) << " " << pointPairSV::maxHash << std::endl;
					hashTable[index].push_back(PPF);
				}
				else
				{

					// alpha_m should be equal (i.e. same bin)
					//if(abs(hashTable[index][pointPairIndex].alpha-alpha)>=angleStep)
					//	break;

					//std::cout << " alpha hash: " << hashTable[index][pointPairIndex].alpha*RAD_TO_DEG << " alpha new: " << alpha*RAD_TO_DEG << " " << (hashTable[index][pointPairIndex].alpha-alpha)*RAD_TO_DEG <<" " << std::atan2(1,-1)*RAD_TO_DEG << " " << std::atan2(-1,1)*RAD_TO_DEG <<std::endl;
					++hashTable[index][pointPairIndex].weight;
					//std::cout  << hashTable[index][pointPairIndex].weight << std::endl;
					++discardedPointPairs;
					++discardedNow;
					//if(abs(hashTable[index][pointPairIndex].alpha-alpha)>=angleStep && ola==50)
					
					/*if((mr->z!=mi->z))
					{
//std::cout << " alpha hash: " << hashTable[index][pointPairIndex].alpha*RAD_TO_DEG << " alpha new: " << alpha*RAD_TO_DEG << " " <<std::endl;
						pcl::PointCloud<pcl::PointNormal>::Ptr pointPairsCloud(new pcl::PointCloud<pcl::PointNormal>);
						pointPairsCloud->push_back(*mr);
						pointPairsCloud->push_back(*mi);
						pointPairsCloud->push_back(hashTable[index][pointPairIndex].referencePoint);
						pointPairsCloud->push_back(hashTable[index][pointPairIndex].otherPoint);
						std::cout << "DISTANCE 1: " << sqrt((mr->x-mi->x)*(mr->x-mi->x) + (mr->y-mi->y)*(mr->y-mi->y)+(mr->z-mi->z)*(mr->z-mi->z)) << std::endl;
						std::cout << "DISTANCE 2: " << sqrt(
(hashTable[index][pointPairIndex].referencePoint.x-hashTable[index][pointPairIndex].otherPoint.x)*
(hashTable[index][pointPairIndex].referencePoint.x-hashTable[index][pointPairIndex].otherPoint.x)+
(hashTable[index][pointPairIndex].referencePoint.y-hashTable[index][pointPairIndex].otherPoint.y)*
(hashTable[index][pointPairIndex].referencePoint.y-hashTable[index][pointPairIndex].otherPoint.y)+
(hashTable[index][pointPairIndex].referencePoint.z-hashTable[index][pointPairIndex].otherPoint.z)*
(hashTable[index][pointPairIndex].referencePoint.z-hashTable[index][pointPairIndex].otherPoint.z)) << std::endl;
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloudRGB(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
						pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
						for(pcl::PointCloud<pcl::PointNormal>::iterator mp=modelCloud->begin(); mp<modelCloud->end(); ++mp)
						{
							if(mp==mr||mp==mi)
								continue;
							if(mp->x==hashTable[index][pointPairIndex].referencePoint.x && mp->y==hashTable[index][pointPairIndex].referencePoint.y && mp->z==hashTable[index][pointPairIndex].referencePoint.z)
								continue;
							if(mp->x==hashTable[index][pointPairIndex].otherPoint.x && mp->y==hashTable[index][pointPairIndex].otherPoint.y && mp->z==hashTable[index][pointPairIndex].otherPoint.z)
								continue;

							//addPointRGB(*mp, pointCloudRGB, 0);
							addPointRGB(*mp, pointCloudRGB, 0);
						}
						//REF POINT
						//addPointRGB(*mr, pointCloudRGB, 1);
						addPointRGB(*mi, pointCloudRGB, 1);

						//addPointRGB(hashTable[index][pointPairIndex].referencePoint, pointCloudRGB, 2);
						addPointRGB(hashTable[index][pointPairIndex].otherPoint, pointCloudRGB, 2);
						
						boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(pointCloudRGB);
						viewer->addSphere (*mr, distanceStep/2, 1.0, 0.0, 0.0, "sphere_ref_new");
						viewer->addSphere (hashTable[index][pointPairIndex].referencePoint, distanceStep/2, 0.0, 1.0, 0.0, "sphere_ref_model");
						while (!viewer->wasStopped ())
						{
							viewer->spinOnce (100);
							boost::this_thread::sleep (boost::posix_time::microseconds (100000));
						}

*/


	/*pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPairsCloudBeforeTransformOne(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPairsCloudAfterTransformOne(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	Eigen::Vector3f _refPointOne=hashTable[index][pointPairIndex].referencePoint.getVector3fMap();
	Eigen::Vector3f normalRefOne=hashTable[index][pointPairIndex].referencePoint.getNormalVector3fMap();
	normalRefOne.normalize();

	// Get transformation from local frame to global frame	
	Eigen::AngleAxisf rotationOneToGlobal(acosf (normalRefOne.dot (Eigen::Vector3f::UnitX ())), normalRefOne.cross (Eigen::Vector3f::UnitX ()).normalized ());
	Eigen::Affine3f transformOneToGlobal = Eigen::Translation3f( rotationOneToGlobal * ((-1) * _refPointOne)) * rotationOneToGlobal;
	Eigen::Affine3f completeOne = Eigen::AngleAxisf(hashTable[index][pointPairIndex].alpha, Eigen::Vector3f::UnitX ()) * transformOneToGlobal;

	addPointRGB(hashTable[index][pointPairIndex].referencePoint,pointPairsCloudBeforeTransformOne,2);
	addPointRGB(hashTable[index][pointPairIndex].otherPoint,pointPairsCloudBeforeTransformOne,2);

	//pcl::transformPointCloudWithNormals(*pointPairsCloudBeforeTransformOne, *pointPairsCloudAfterTransformOne, completeOne);
	pcl::transformPointCloudWithNormals(*pointPairsCloudBeforeTransformOne, *pointPairsCloudAfterTransformOne, transformOneToGlobal);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4 = normalsVis(pointPairsCloudAfterTransformOne);
	while (!viewer4->wasStopped ())
	{
		viewer4->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}



	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPairsCloudBeforeTransformTwo(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointPairsCloudAfterTransformTwo(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	Eigen::Vector3f _refPointTwo=mr->getVector3fMap();
	Eigen::Vector3f normalRefTwo=mr->getNormalVector3fMap();
	normalRefTwo.normalize();

	// Get transformation from local frame to global frame	
	Eigen::AngleAxisf rotationTwoToGlobal(acosf (normalRefTwo.dot (Eigen::Vector3f::UnitX ())), normalRefTwo.cross (Eigen::Vector3f::UnitX ()).normalized ());
	Eigen::Affine3f transformTwoToGlobal = Eigen::Translation3f( rotationTwoToGlobal * ((-1) * _refPointTwo)) * rotationTwoToGlobal;
	Eigen::Affine3f completeTwo = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX ()) * transformTwoToGlobal;

	addPointRGB(*mr,pointPairsCloudBeforeTransformTwo,1);
	addPointRGB(*mi,pointPairsCloudBeforeTransformTwo,1);

	//pcl::transformPointCloudWithNormals(*pointPairsCloudBeforeTransformTwo, *pointPairsCloudAfterTransformTwo, completeTwo);
	pcl::transformPointCloudWithNormals(*pointPairsCloudBeforeTransformTwo, *pointPairsCloudAfterTransformTwo, transformTwoToGlobal);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer5 = normalsVis(pointPairsCloudAfterTransformTwo);
	while (!viewer5->wasStopped ())
	{
		viewer5->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	*pointPairsCloudAfterTransformTwo+=*pointPairsCloudAfterTransformOne;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer6 = normalsVis(pointPairsCloudAfterTransformTwo);
	while (!viewer6->wasStopped ())
	{
		viewer6->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
*/

					//}
					
				}
			}
			++idRef;
			std::cout << idRef << " discarded now: " << discardedNow << " discarded total: " << discardedPointPairs << std::endl;
		}

		std::cout << "  remaining point pairs:" << remainingPointPairs << " (" << (remainingPointPairs/(remainingPointPairs+discardedPointPairs))*100 << "%)" << std::endl;
		std::cout << "  discarded point pairs:" << discardedPointPairs << " (" << (discardedPointPairs/(remainingPointPairs+discardedPointPairs))*100 << "%)" << std::endl;
		//std::cout << "ola: " << coco << std::endl;

	}
	else
	{

		//std::cout << modelCloud->size() << std::endl;
		int i=0;
		for(mr=modelCloud->begin(); mr<modelCloud->end(); ++mr)
		{
			Eigen::Vector3f newPoint=mr->getVector3fMap();
			Eigen::Vector3f newNormal=mr->getNormalVector3fMap ();
			Eigen::Vector3f cross=newNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

			Eigen::AngleAxisf rotationToGlobal;
			if(newNormal.isApprox(Eigen::Vector3f::UnitX (),0.0001))
			{
				rotationToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
			}
			else
			{
				cross=newNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

				if (isnan(cross[0]))
				{
					std::cout << "YA"<< std::endl;
					exit(-1);
					rotationToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
				}
				else
				{
					rotationToGlobal=Eigen::AngleAxisf(acosf (newNormal.dot (Eigen::Vector3f::UnitX ())),cross);
				}
			}

			Eigen::Affine3f transformToGlobal = Eigen::Translation3f ( rotationToGlobal* ((-1)*newPoint)) * rotationToGlobal;
			//std::cout << ++i << std::endl;
			// Add reference point to point cloud
			for(mi=modelCloud->begin();mi<modelCloud->end(); ++mi)
			{
				// If same point... continue...
				if(mr==mi)
				{
					continue;
				}

				float dist=sqrt((mr->x-mi->x)*
					  	  (mr->x-mi->x)+
					  	  (mr->y-mi->y)*
					  	  (mr->y-mi->y)+
					  	  (mr->z-mi->z)*
					          (mr->z-mi->z));
				
				// Compute PPF
				pointPairSV PPF=pointPairSV(*mr,*mi,transformToGlobal,idRef);

				// Compute index
				int index=PPF.getHash(*mi,distanceStepInverted);

				if(index >=(int)hashTable.size())
				{
					std::cout << "bolas " << std::endl;

				}
				//std::cout << PPF.getHash(*mi,distanceStepInverted) << " " << pointPairSV::maxHash << std::endl;
				hashTable[index].push_back(PPF);
			}

			idRef++;
		}
	}		
	std::cout << "features_perdidas_por_erro_de_amostragem: "<<features_perdidas_por_erro_de_amostragem << std::endl;
	std::cout << "  Hash table size: " << hashTable.size() << std::endl;
    //seeHashTableEntropy();
	//exit(-1);
}

//int objectModelSV::computeIndexDebug(const Eigen::Vector4f& feature)
//{
//	Eigen::Vector4d discreteFeature;
//
//	// Discretize the output
//	discreteFeature[0]=floor(feature[0]*distanceStepInverted);
//	if(discreteFeature[0]==distanceBins) --discreteFeature[0];
//
//	discreteFeature[1]=lutCosToAngle[ floor( (feature[1]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	discreteFeature[2]=lutCosToAngle[ floor( (feature[2]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	discreteFeature[3]=lutCosToAngle[ floor( (feature[3]+1.0) * static_cast<float>(cosResolution)*0.5) ];
//
//	std::cout << "discrete feature:" << discreteFeature << std::endl;
//	std::cout << " ..." << floor( (feature[3]+1.0) * static_cast<float>(cosResolution)*0.5) << std::endl;
//	std::cout << " ..." << feature[3]+1.0<< std::endl;
//	return 	(discreteFeature[0]+(discreteFeature[1]*distanceBins)+(discreteFeature[2]*distanceBins*angleBins)+(discreteFeature[3]*distanceBins*angleBins*angleBins));
//}

