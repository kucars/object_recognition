#include "objectrecognition_sv/pose_estimation.h"
  #include "boost/date_time/posix_time/posix_time.hpp"
//clock_t timeIni;
//double timeEnd;
//void Tic(){ timeIni=clock(); }
//void Tac(){ timeEnd = (double)(clock()-timeIni);} ///(double)CLOCKS_PER_SEC; //std::cout << timeEnd*1000 << "ms (" << 1/timeEnd << " fps)" << std::endl;}




boost::posix_time::ptime timeIni;
boost::posix_time::time_duration timeEnd_;
double timeEnd;
void Tic()
{
 timeIni = boost::posix_time::microsec_clock::local_time();
}

void Tac()
{
 	timeEnd_ = boost::posix_time::microsec_clock::local_time() - timeIni	;
	timeEnd=(double)timeEnd_.total_microseconds() ;

	//std::cout << timeEnd<< std::endl;
}




   //boost::this_thread::sleep(boost::posix_time::millisec(500));
  /* boost::posix_time::ptime t2 = boost::posix_time::second_clock::local_time();
   boost::posix_time::time_duration diff = t2 - t1;
   std::cout << diff.total_milliseconds() << std::endl;

   boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();
   boost::this_thread::sleep(boost::posix_time::millisec(500));
   boost::posix_time::ptime mst2 = boost::posix_time::microsec_clock::local_time();
   boost::posix_time::time_duration msdiff = mst2 - mst1;
   std::cout << msdiff.total_milliseconds() << std::endl;
 Sent at 11:46 AM on Tuesday*/
 





#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

class testsData
{
	public:
		int size_alpha;
		int size_points;
		std::vector < std::vector<int> > accumulator;

		testsData(const std::vector < std::vector<int> > & accumulator_): accumulator(accumulator_)
		{
			size_alpha=accumulator.size();
			size_points=accumulator[0].size();
		}
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & size_alpha;
			ar & size_points;
			ar & accumulator;
		}
};

void saveData2(const char* filename, const testsData & accumulator)
{
    // create and open a character archive for output
   	std::ofstream ofs(filename);
    // save data to archive
    {
		boost::archive::text_oarchive oa(ofs);
		// write class instance to archive
		//oa << accumulator.size();
		//oa << accumulator[0].size();
		oa << accumulator;
    	// archive and stream closed when destructors are called
    }
}


void saveDataOldWay(const char* filename, const std::vector < std::vector<int> > & accumulator)
{
	std::cout << "saving data to : " << filename << std::endl;
    	// create and open a character archive for output
	std::ofstream myfile;
	myfile.open (filename);

	for(int i=0; i< accumulator.size();++i)
	{
		for(int j=0; j< accumulator[i].size();++j)
		{
			  myfile << accumulator[i][j] << " ";
		}
		myfile << "\n";

	}
	  myfile.close();

}

void saveData(const char* filename, const std::vector < std::vector<int> > & accumulator)
{
    	// create and open a character archive for output
   	std::ofstream ofs(filename);	
    	// save data to archive
    	{
		boost::archive::text_oarchive oa(ofs);
		// write class instance to archive
		//oa << accumulator.size();
		//oa << accumulator[0].size();
		oa << accumulator;
    		// archive and stream closed when destructors are called
    	}
}

/*void poseEstimationSV::accumulatorToImage()
{
    // Allocate a 1-channel byte image
	int lines = accumulator.size();
	int columns=accumulator[0].size() ;

	//std::cout << "LINES: " << lines << std::endl;
	//std::cout << "COLUMNS: " << columns << std::endl;

	IplImage* accumulator_image=cvCreateImage(cvSize(columns, lines),IPL_DEPTH_8U,1);
  cv_bridge::CvImagePtr cv_ptr;
	for(unsigned int line_index=0; line_index < lines; ++line_index)
	{
		for(unsigned int column_index=0; column_index < columns; ++column_index)
		{
			CvScalar s;
			s.val[0]=(int)floor(255*accumulator[line_index][column_index]);
			std::cout << s.val[0] << std::endl;
			cvSet2D(accumulator_image, line_index, column_index ,s); // set the (i, j) pixel value
		}
	}
    cv::waitKey(300);
  cv_ptr=accumulator_image;
	try
	{
		accumulator_image_pub_.publish(cv_ptr->toImageMsg());
		ROS_INFO("Graspability map image published.");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error publishing image.");
	}

}*/







poseEstimationSV::poseEstimationSV(objectModelPtr inputModel)
{	
	///////////////////////////
   	// Initialize parameters //
	///////////////////////////

	model=inputModel;

	////////////////////////////////////////////////////////
	// Allocate space for several accumulators (parallel) //
	////////////////////////////////////////////////////////		
	accumulatorParallelAux.resize(omp_get_num_procs());
	for(int i=0;i < omp_get_num_procs(); ++i)
	{
		// Allocate space for reference points (lines)
		accumulatorParallelAux[i].resize(model->modelCloud->size());
		for(std::vector<std::vector<int> >::iterator refIt=accumulatorParallelAux[i].begin();refIt < accumulatorParallelAux[i].end(); ++refIt)
		{
			// Allocate space for poses (columns)
			refIt->resize(pointPair::angleBins);
		}
	}

	////////////////////////////////////////
	// Allocate space for the accumulator //
	////////////////////////////////////////

	// Allocate space for reference points (lines)
	accumulator.resize(model->modelCloud->size());
	for(std::vector<std::vector<int> >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
	{
		// Allocate space for poses (columns)
		refIt->resize(pointPair::angleBins);
	}

	//////////////////////////////////////////////
	// Allocate space for the best poses vector //
	//////////////////////////////////////////////

        bestPoses.reserve(model->modelCloud->size()*pointPair::angleBins);
}

poseEstimationSV::~poseEstimationSV()
{	
	///////////////////////////////////////////
	// Deallocate space for the accumulators //
	///////////////////////////////////////////

	for(int i=0;i < omp_get_num_procs(); ++i)
	{
		for(std::vector<std::vector<int> >::iterator refIt=accumulatorParallelAux[i].begin();refIt < accumulatorParallelAux[i].end(); ++refIt)
		{
			// Deallocate space allocated for poses (columns)
			refIt->clear();
		}
		// Deallocate space allocated for reference points (lines)
		accumulatorParallelAux[i].clear();
	}
	accumulatorParallelAux.clear();

	//////////////////////////////////////////
	// Deallocate space for the accumulator //
	//////////////////////////////////////////

	for(std::vector<std::vector<int> >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
	{
		// Deallocate space allocated for poses (columns)
		refIt->clear();
	}
	// Deallocate space allocated for reference points (lines)
	accumulator.clear();
}

// METHOD THAT RECEIVES POINT CLOUDS
std::vector<cluster> poseEstimationSV::poseEstimationCore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
Tic();
	int bestPoseAlpha;
	int bestPosePoint;
	int bestPoseVotes;
	
	pcl::PointIndices normals_nan_indices;
	pcl::ExtractIndices<pcl::PointNormal> nan_extract;

	float alpha;
	unsigned int alphaBin,index;
	// Iterators
	std::vector<int>::iterator sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSV>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;
	Eigen::Vector3f _pointTwoTransformed;

	std::cout<< "\tCloud size: " << cloud->size() << endl;

	//////////////////////////////////////////////
	// Downsample point cloud using a voxelgrid //
	//////////////////////////////////////////////

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZ> ());
  	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (model->distanceStep,model->distanceStep,model->distanceStep);
  	sor.filter (*cloudDownsampled);
	std::cout<< "\tCloud size after downsampling: " << cloudDownsampled->size() << endl;

	// Compute point cloud normals (using cloud before downsampling information)
	std::cout<< "\tCompute normals... ";
	cloudNormals=model->computeSceneNormals(cloudDownsampled);
	std::cout<< "Done" << endl;

    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(cloudNormals);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/

	/*boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(model->modelCloud);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/
	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i) 
	{
		if (isnan(cloudNormals->points[i].normal[0]) || isnan(cloudNormals->points[i].normal[1]) || isnan(cloudNormals->points[i].normal[2]))
		{
	   		normals_nan_indices.indices.push_back(i);
		}
	}

	nan_extract.setInputCloud(cloudNormals);
	nan_extract.setIndices(boost::make_shared<pcl::PointIndices> (normals_nan_indices));
	nan_extract.setNegative(true);
	nan_extract.filter(*cloudWithNormalsDownSampled);


	std::cout<< "\tCloud size after removing NaN normals: " << cloudWithNormalsDownSampled->size() << endl;

	////////////////////////////////////////////////////
	// Extract random reference points from the scene //
	////////////////////////////////////////////////////

	//pcl::RandomSample< pcl::PointCloud<pcl::PointNormal> > randomSampler;
	//randomSampler.setInputCloud(cloudWithNormalsDownSampled);
	// Create the filtering object
	int numberOfPoints=(int) (cloudWithNormalsDownSampled->size () )*referencePointsPercentage;
	int totalPoints=(int) (cloudWithNormalsDownSampled->size ());
	std::cout << "\tUniform sample a set of " << numberOfPoints << "(" << referencePointsPercentage*100 <<  "%)... ";
	referencePointsIndices->indices.clear();
	extractReferencePointsUniform(referencePointsPercentage,totalPoints);
	std::cout << "Done" << std::endl;
	//std::cout << referencePointsIndices->indices.size() << std::endl;

	//////////////
	// Votation //
	//////////////

	std::cout<< "\tVotation... ";
        bestPoses.clear();
	for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		Eigen::Vector3f scenePoint=cloudWithNormalsDownSampled->points[*sr].getVector3fMap();
		Eigen::Vector3f sceneNormal=cloudWithNormalsDownSampled->points[*sr].getNormalVector3fMap ();

		// Get transformation from scene frame to global frame
		Eigen::Vector3f cross=sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

		Eigen::Affine3f rotationSceneToGlobal;
		if(isnan(cross[0]))
		{
			rotationSceneToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
		}
		else
			rotationSceneToGlobal=Eigen::AngleAxisf(acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),cross);

		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		for(std::vector<std::vector<int> >::iterator accumulatorIt=accumulator.begin();accumulatorIt < accumulator.end(); ++accumulatorIt)
		{
			std::fill(accumulatorIt->begin(),accumulatorIt->end(),0); 
		}
		
		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[*sr].x==si->x) && (cloudWithNormalsDownSampled->points[*sr].y==si->y) && (cloudWithNormalsDownSampled->points[*sr].z==si->z))
			{
				continue;
			}	

			// Compute PPF
			pointPairSV PPF=pointPairSV(cloudWithNormalsDownSampled->points[*sr],*si,transformSceneToGlobal);

			// Compute index
			index=PPF.getHash(*si,model->distanceStepInverted);

			// If distance between point pairs is bigger than the maximum for this model, skip point pair
			if(index>=pointPairSV::maxHash)
			{
				//std::cout << "DEBUG" << std::endl;
				continue;
			}

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue; 

			// Iterate over similar point pairs
			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{
				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-PPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;

				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication

				if(alphaBin>=pointPair::angleBins)
				{	
					alphaBin=0;
				}

				accumulator[sameFeatureIt->id][alphaBin]+=sameFeatureIt->weight;
			}
		}

		// Choose best pose (highest peak on the accumulator[peak with more votes])
		bestPosePoint=0;
		bestPoseAlpha=0;
		bestPoseVotes=0;
		
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulator[p][a]>bestPoseVotes)
				{
					bestPoseVotes=accumulator[p][a];
					bestPosePoint=p;
					bestPoseAlpha=a;
				}
			}
		}

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			// Compute and store transformation from model to scene
			//boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(model->modelCloud->points[bestPosePoint],transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
			//boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
			//bestPoses.push_back(bestPose);
			bestPoses.push_back(pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
		}
		else 
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulator[bestPosePoint][bestPoseAlpha]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulator[p][a]>=accumulatorPeakThreshold*bestPoseVotes)
				{
					// Compute and store transformation from model to scene
					//boost::shared_ptr<pose> bestPose(new pose( accumulator[p][a],model->modelToScene(model->modelCloud->points[p],transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI ) ));
					//boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));

					//bestPoses.push_back(bestPose);

                                        bestPoses.push_back(pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
				}
			}
		}
	}
	std::cout << "Done" << std::endl;

	if(bestPoses.size()==0)
	{

		clusters.clear();
		return clusters;
	}

	//////////////////////
	// Compute clusters //
	//////////////////////
Tac();
	std::cout << "\tCompute clusters... ";
Tic();
	clusters=poseClustering(bestPoses);
Tac();
	std::cout << "Done" << std::endl;

	return clusters;
}

// METHOD THAT RECEIVES POINT CLOUDS (OPEN MP)
std::vector<cluster> poseEstimationSV::poseEstimationCore_openmp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
Tic();
	std::vector <std::vector < pose > > bestPosesAux;
	bestPosesAux.resize(omp_get_num_procs());

	//int bestPoseAlpha;
	//int bestPosePoint;
	//int bestPoseVotes;
	
	Eigen::Vector3f scenePoint;
	Eigen::Vector3f sceneNormal;


	pcl::PointIndices normals_nan_indices;
	pcl::ExtractIndices<pcl::PointNormal> nan_extract;

	float alpha;
	unsigned int alphaBin,index;
	// Iterators
	//unsigned int sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSV>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;
	Eigen::Vector3f _pointTwoTransformed;
	std::cout<< "\tCloud size: " << cloud->size() << endl;
	//////////////////////////////////////////////
	// Downsample point cloud using a voxelgrid //
	//////////////////////////////////////////////

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZ> ());
  	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (model->distanceStep,model->distanceStep,model->distanceStep);
  	sor.filter (*cloudDownsampled);
	std::cout<< "\tCloud size after downsampling: " << cloudDownsampled->size() << endl;

	// Compute point cloud normals (using cloud before downsampling information)
	std::cout<< "\tCompute normals... ";
	cloudNormals=model->computeSceneNormals(cloudDownsampled);
	std::cout<< "Done" << endl;

	/*boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(cloudFilteredNormals);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/

	/*boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(model->modelCloud);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/
	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	////////////////////////////////////////////////fa//////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i) 
	{
		if (isnan(cloudNormals->points[i].normal[0]) || isnan(cloudNormals->points[i].normal[1]) || isnan(cloudNormals->points[i].normal[2]))
		{
	   		normals_nan_indices.indices.push_back(i);
		}
	}

	nan_extract.setInputCloud(cloudNormals);
	nan_extract.setIndices(boost::make_shared<pcl::PointIndices> (normals_nan_indices));
	nan_extract.setNegative(true);
	nan_extract.filter(*cloudWithNormalsDownSampled);
	std::cout<< "\tCloud size after removing NaN normals: " << cloudWithNormalsDownSampled->size() << endl;


	/////////////////////////////////////////////
	// Extract reference points from the scene //
	/////////////////////////////////////////////

	//pcl::RandomSample< pcl::PointCloud<pcl::PointNormal> > randomSampler;
	//randomSampler.setInputCloud(cloudWithNormalsDownSampled);
	// Create the filtering object
	int numberOfPoints=(int) (cloudWithNormalsDownSampled->size () )*referencePointsPercentage;
	int totalPoints=(int) (cloudWithNormalsDownSampled->size ());
	std::cout << "\tUniform sample a set of " << numberOfPoints << "(" << referencePointsPercentage*100 <<  "%)... ";
	referencePointsIndices->indices.clear();
	extractReferencePointsUniform(referencePointsPercentage,totalPoints);
	std::cout << "Done" << std::endl;
	//std::cout << referencePointsIndices->indices.size() << std::endl;

	//////////////
	// Votation //
	//////////////

	std::cout<< "\tVotation... ";

	omp_set_num_threads(omp_get_num_procs());
	//omp_set_num_threads(1);
	//int iteration=0;

        bestPoses.clear();
	#pragma omp parallel for private(alpha,alphaBin,alphaScene,sameFeatureIt,index,feature,si,_pointTwoTransformed) //reduction(+:iteration)  //nowait
	for(unsigned int sr=0; sr < referencePointsIndices->indices.size(); ++sr)
	{
	
		//++iteration;
		//std::cout << "iteration: " << iteration << " thread:" << omp_get_thread_num() << std::endl;
		//printf("Hello from thread %d, nthreads %d\n", omp_get_thread_num(), omp_get_num_threads());
		scenePoint=cloudWithNormalsDownSampled->points[referencePointsIndices->indices[sr]].getVector3fMap();
		sceneNormal=cloudWithNormalsDownSampled->points[referencePointsIndices->indices[sr]].getNormalVector3fMap();

		// Get transformation from scene frame to global frame
		Eigen::Vector3f cross=sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

		Eigen::Affine3f rotationSceneToGlobal;
		if(isnan(cross[0]))
		{
			rotationSceneToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
		}
		else
			rotationSceneToGlobal=Eigen::AngleAxisf(acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),cross);

		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		for(std::vector<std::vector<int> >::iterator accumulatorIt=accumulatorParallelAux[omp_get_thread_num()].begin();accumulatorIt < accumulatorParallelAux[omp_get_thread_num()].end(); ++accumulatorIt)
		{
			std::fill(accumulatorIt->begin(),accumulatorIt->end(),0); 
		}
		

		//std::cout << std::endl;
		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[referencePointsIndices->indices[sr]].x==si->x) && (cloudWithNormalsDownSampled->points[referencePointsIndices->indices[sr]].y==si->y) && (cloudWithNormalsDownSampled->points[referencePointsIndices->indices[sr]].z==si->z))
			{
				//std::cout << si->x << " " << si->y << " " << si->z << std::endl;
				continue;
			}	

			// Compute PPF
			pointPairSV PPF=pointPairSV(cloudWithNormalsDownSampled->points[sr],*si, transformSceneToGlobal);

			// Compute index
			index=PPF.getHash(*si,model->distanceStepInverted);

			// If distance between point pairs is bigger than the maximum for this model, skip point pair
			if(index>pointPairSV::maxHash)
			{
				//std::cout << "DEBUG" << std::endl;
				continue;
			}

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue; 

			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{
				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-PPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;
				//std::cout << "alpha after: " << alpha*RAD_TO_DEG << std::endl;
				//std::cout << "alpha after2: " << (alpha+PI)*RAD_TO_DEG << std::endl;
				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication
				//std::cout << "angle1: " << alphaBin << std::endl;
           			/*alphaBin = static_cast<unsigned int> (floor (alpha) + floor (PI *poseAngleStepInverted));
				std::cout << "angle2: " << alphaBin << std::endl;*/
				//alphaBin=static_cast<unsigned int> ( floor(alpha*poseAngleStepInverted) + floor(PI*poseAngleStepInverted) );
				if(alphaBin>=pointPair::angleBins)
				{	
					alphaBin=0;
					//ROS_INFO("naoooo");
					//exit(1);
				}

//#pragma omp critical
//{std::cout << index <<" "<<sameFeatureIt->id << " " << alphaBin << " " << omp_get_thread_num() << " " << accumulatorParallelAux[omp_get_thread_num()][sameFeatureIt->id][alphaBin] << std::endl;}

				accumulatorParallelAux[omp_get_thread_num()][sameFeatureIt->id][alphaBin]+=sameFeatureIt->weight;
			}
		}
		//ROS_INFO("DISTANCE:%f DISTANCE SQUARED:%f", model->maxModelDist, model->maxModel

		// Choose best pose (highest peak on the accumulator[peak with more votes])

		int bestPoseAlpha=0;
		int bestPosePoint=0;
		int bestPoseVotes=0;

		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulatorParallelAux[omp_get_thread_num()][p][a]>bestPoseVotes)
				{
					bestPoseVotes=accumulatorParallelAux[omp_get_thread_num()][p][a];
					bestPosePoint=p;
					bestPoseAlpha=a;
				}
			}
		}

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			// Compute and store transformation from model to scene
			//boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(model->modelCloud->points[bestPosePoint],transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));

			bestPosesAux[omp_get_thread_num()].push_back(pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
			//bestPoses.push_back(bestPose);

			//std::cout << bestPosesAux[omp_get_thread_num()].size() <<" " <<omp_get_thread_num()<< std::endl;
		}
		else 
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulatorParallelAux[omp_get_thread_num()][bestPosePoint][bestPoseAlpha]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulatorParallelAux[omp_get_thread_num()][p][a]>=accumulatorPeakThreshold*bestPoseVotes)
				{
					// Compute and store transformation from model to scene
					//boost::shared_ptr<pose> bestPose(new pose( accumulatorParallelAux[omp_get_thread_num()][p][a],model->modelToScene(model->modelCloud->points[p],transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI ) ));


					//bestPoses.push_back(bestPose);
					bestPosesAux[omp_get_thread_num()].push_back(pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
					//std::cout << bestPosesAux[omp_get_thread_num()].size() <<" " <<omp_get_thread_num()<< std::endl;
				}
			}
		}
	}

	std::cout << "Done" << std::endl;


	for(int i=0; i<omp_get_num_procs(); ++i)
	{
		for(unsigned int j=0; j<bestPosesAux[i].size(); ++j)
			bestPoses.push_back(bestPosesAux[i][j]);
	}
	std::cout << "\thypothesis number: " << bestPoses.size() << std::endl << std::endl;

	if(bestPoses.size()==0)
	{
		clusters.clear();
		return clusters;
	}

	
	//////////////////////
	// Compute clusters //
	//////////////////////
Tac();
	std::cout << "\tCompute clusters... ";
Tic();
	clusters=poseClustering(bestPoses);
Tac();
	std::cout << "Done" << std::endl;

	return clusters;
}

// METHOD THAT RECEIVES SURFLET CLOUDS (USED FOR MIAN DATASED)
std::vector<cluster> poseEstimationSV::poseEstimationCore(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{

	int bestPoseAlpha;
	int bestPosePoint;
	int bestPoseVotes;
	
	pcl::PointIndices normals_nan_indices;


	float alpha;
	unsigned int alphaBin,index;
	// Iterators
	std::vector<int>::iterator sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSV>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;

	/*std::cout << "\tDownsample dense surflet cloud... " << std::endl;
	cout << "\t\tSurflet cloud size before downsampling: " << cloud->size() << endl;
 	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (model->distanceStep,model->distanceStep,model->distanceStep);
  	sor.filter (*cloudNormals);
	cout << "\t\tSurflet cloud size after downsampling: " << cloudNormals->size() << endl;
	std::cout<< "\tDone" << std::endl;*/

	cloudNormals=cloud;
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(cloudNormals);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/
	/*viewer2 = objectModel::viewportsVis(cloudNormals);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/
	/*std::cout<< "  Re-estimate normals... " << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudPoint(new pcl::PointCloud<pcl::PointXYZ>);
	for(si=cloudNormals->begin(); si<cloudNormals->end(); ++si)
	{
		modelCloudPoint->push_back(pcl::PointXYZ(si->x,si->y,si->z));
	}
	model->computeNormals(modelCloudPoint,cloudNormals);
	std::cout << "  Done" << std::endl;*/

	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i) 
	{
		if (isnan(cloudNormals->points[i].normal[0]) || isnan(cloudNormals->points[i].normal[1]) || isnan(cloudNormals->points[i].normal[2]))
		{
	   		normals_nan_indices.indices.push_back(i);
		}
	}


	pcl::ExtractIndices<pcl::PointNormal> nan_extract;
	nan_extract.setInputCloud(cloudNormals);
	nan_extract.setIndices(boost::make_shared<pcl::PointIndices> (normals_nan_indices));
	nan_extract.setNegative(true);
	nan_extract.filter(*cloudWithNormalsDownSampled);
	std::cout<< "\tCloud size after removing NaN normals: " << cloudWithNormalsDownSampled->size() << endl;

	//////////////
	// VOTATION //
	//////////////

	

	/////////////////////////////////////////////
	// Extract reference points from the scene //
	/////////////////////////////////////////////
	//pcl::RandomSample< pcl::PointCloud<pcl::PointNormal> > randomSampler;
	//randomSampler.setInputCloud(cloudWithNormalsDownSampled);

	int numberOfPoints=(int) (cloudWithNormalsDownSampled->size () )*referencePointsPercentage;
	int totalPoints=(int) (cloudWithNormalsDownSampled->size ());
	std::cout << "\tUniform sample a set of " << numberOfPoints << "(" << referencePointsPercentage*100 <<  "%)... ";
	referencePointsIndices->indices.clear();
	extractReferencePointsUniform(referencePointsPercentage,totalPoints);
	std::cout << "Done" << std::endl;
	//std::cout << referencePointsIndices->indices.size() << std::endl;
	//pcl::PointCloud<pcl::PointNormal>::Ptr testeCloud=pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>);
	/*for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		testeCloud->push_back(cloudWithNormalsDownSampled->points[*sr]);
	}*/
	//std::cout <<  totalPoints << std::endl;
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 = objectModel::viewportsVis(cloudWithNormalsDownSampled);

  	while (!viewer2->wasStopped ())
  	{
   		viewer2->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}*/
	//////////////
	// Votation //
	//////////////
	// Clear all transformations stored
	std::cout<< "\tVotation... ";
  //std::vector<int>::size_type sz;

  bestPoses.clear();
  //sz = bestPoses.capacity();

  std::vector<int> matches_per_feature;



	std::vector<double> scene_to_global_time;
	std::vector<double> reset_accumulator_time;
	std::vector<double> ppf_time;
	std::vector<double> hash_time;
	std::vector<double> matching_time;
	std::vector<double> get_best_peak_time;

	//Tic();
	for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.begin()+1; ++sr)
	//for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		Tic();
		Eigen::Vector3f scenePoint=cloudWithNormalsDownSampled->points[*sr].getVector3fMap();
		Eigen::Vector3f sceneNormal=cloudWithNormalsDownSampled->points[*sr].getNormalVector3fMap ();


		Eigen::Vector3f cross=sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized();
		Eigen::Affine3f rotationSceneToGlobal;

		// Get transformation from scene frame to global frame
		if(sceneNormal.isApprox(Eigen::Vector3f::UnitX (),0.00001))
		{
			rotationSceneToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
		}
		else
		{
			cross=sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized();

			if (isnan(cross[0]))
			{
				std::cout << "YA"<< std::endl;
				exit(-1);
				rotationSceneToGlobal=Eigen::AngleAxisf(0.0,Eigen::Vector3f::UnitX ());
			}
			else
			{
				rotationSceneToGlobal=Eigen::AngleAxisf(acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),cross);
			}
		}

		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		Tac();
		scene_to_global_time.push_back(timeEnd);
		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		Tic();
		for(std::vector<std::vector<int> >::iterator accumulatorIt=accumulator.begin();accumulatorIt < accumulator.end(); ++accumulatorIt)
		{
			std::fill(accumulatorIt->begin(),accumulatorIt->end(),0); 
		}
		Tac();
		reset_accumulator_time.push_back(timeEnd);

		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[*sr].x==si->x) && (cloudWithNormalsDownSampled->points[*sr].y==si->y) && (cloudWithNormalsDownSampled->points[*sr].z==si->z))
			{
				continue;
			}	
			float squaredDist=(cloudWithNormalsDownSampled->points[*sr].x-si->x)*
					  (cloudWithNormalsDownSampled->points[*sr].x-si->x)+
					  (cloudWithNormalsDownSampled->points[*sr].y-si->y)*
					  (cloudWithNormalsDownSampled->points[*sr].y-si->y)+
					  (cloudWithNormalsDownSampled->points[*sr].z-si->z)*
					  (cloudWithNormalsDownSampled->points[*sr].z-si->z);
			if(squaredDist>model->maxModelDistSquared)
			{
				continue;
			}
			Tic();
			float dist=sqrt(squaredDist);
			// Compute PPF
			pointPairSV PPF=pointPairSV(cloudWithNormalsDownSampled->points[*sr],*si,transformSceneToGlobal);
			Tac();
			ppf_time.push_back(timeEnd);
			Tic();
			// Compute index
			index=PPF.getHash(*si,model->distanceStepInverted);
			Tac();
			hash_time.push_back(timeEnd);
			// If distance between point pairs is bigger than the maximum for this model, skip point pair
			if(index>=pointPairSV::maxHash)
			{
				//std::cout << "DEBUG" << std::endl;
				continue;
			}

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue; 

			int matches=0; //Tests
			Tic();
			// Iterate over similar point pairs
			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{
				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-PPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;

				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication

				if(alphaBin>=pointPair::angleBins)
				{	
					alphaBin=0;
				}

				accumulator[sameFeatureIt->id][alphaBin]+=sameFeatureIt->weight;
				++matches;//Tests
			}
			Tac();
			matching_time.push_back(timeEnd);
			matches_per_feature.push_back(matches);
		}
		//Tac();

		// Choose best pose (highest peak on the accumulator[peak with more votes])
		bestPosePoint=0;
		bestPoseAlpha=0;
		bestPoseVotes=0;
		Tic();
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulator[p][a]>bestPoseVotes)
				{
					bestPoseVotes=accumulator[p][a];
					bestPosePoint=p;
					bestPoseAlpha=a;
				}
			}
		}
		Tac();
		get_best_peak_time.push_back(timeEnd);

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			// Compute and store transformation from model to scene
			bestPoses.push_back(pose( bestPoseVotes,model->modelToScene(bestPosePoint,transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI) ));
		}
		else 
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulator[bestPosePoint][bestPoseAlpha]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				if(accumulator[p][a]>=accumulatorPeakThreshold*bestPoseVotes)
				{
					bestPoses.push_back(pose( accumulator[p][a],model->modelToScene(p,transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI) ));
				}
			}
		}

   /* if (sz!=bestPoses.capacity()) {
      sz = bestPoses.capacity();
      std::cout << "capacity changed: " << sz << '\n';
      exit(-1);
    }*/

	}
//Tac();
	std::cout << "Done" << std::endl;

	if(bestPoses.size()==0)
	{

		clusters.clear();
		return clusters;
	}


	//////////////////////
	// Compute clusters //
	//////////////////////

	std::cout << "\tCompute clusters... ";
Tic();
	clusters=poseClustering(bestPoses);
	std::cout << "Done" << std::endl;
	clusters[0].voting_time=timeEnd;
	std::cout << timeEnd << " " << clusters[0].voting_time << std::endl;
Tac();
	clusters[0].clustering_time=timeEnd;
	clusters[0].matches_per_feature=matches_per_feature;
  clusters[0].scene_to_global_time=scene_to_global_time;
	clusters[0].reset_accumulator_time=reset_accumulator_time;
	clusters[0].ppf_time=ppf_time;
	clusters[0].hash_time=hash_time;
	clusters[0].matching_time=matching_time;
	clusters[0].get_best_peak_time=get_best_peak_time;

	std::cout << "clusters size:" << clusters.size()<< std::endl;
	return clusters;
}

// CLUSTERING METHOD
std::vector<cluster> poseEstimationSV::poseClustering(std::vector<pose> & bestPoses)
{
	// Sort clusters
	std::sort(bestPoses.begin(), bestPoses.end(), pose::compare);

	std::vector<poseCluster> centroids;
	std::vector<cluster> clusters;    

	float _orientationDistance;	
	float _positionDistance;
	bool found;
	
	int nearestCentroid;

	///////////////////////////////
	// Initialize first centroid //
	///////////////////////////////

	// If there are no poses, return
	if(bestPoses.empty())
		return clusters;

    centroids.reserve(bestPoses.size());

	centroids.push_back(poseCluster(0,bestPoses[0]));

	// For each pose
	for(size_t p=1; p< bestPoses.size(); ++p)	
	{

		found=false;
		for(size_t c=0; c < centroids.size(); ++c)
		{
			// Compute (squared) distance to the cluster center

			_positionDistance=positionDistance(bestPoses[p],bestPoses[centroids[c].poseIndex]);
			if(_positionDistance<=model->halfDistanceStepSquared)
				continue;
			_orientationDistance=orientationDistance(bestPoses[p],bestPoses[centroids[c].poseIndex]);

			// If the cluster is nearer than a threshold add current pose to the cluster
			if(_orientationDistance>=pointPair::angleStepCos) // VER ISTO
			{
				nearestCentroid=c;
				found=true;
				break;
			}		
		}			
			
		if(found)
		{
			//std::cout << " yes:" << "pose:"<< p <<"nearest centroid:" << nearestCentroid << " " << 2*acos(_orientationDistance)*RAD_TO_DEG << " "<< _positionDistance << " " << bestPoses[p].votes << std::endl;
			centroids[nearestCentroid].update(bestPoses[p]);
		}
		else
		{
			//if(2*acos(_orientationDistance)*RAD_TO_DEG< 6.000 && _positionDistance<model->halfDistanceStepSquared)
			//	std::cout << "angle:" << pointPair::angleStep*RAD_TO_DEG  << "angle threshold: " <<2*acos(pointPair::angleStepCos)*RAD_TO_DEG<< " no:" << 2*acos(_orientationDistance)*RAD_TO_DEG << " "<< _positionDistance << std::endl;
			centroids.push_back(poseCluster(p,bestPoses[p]));
		}
		//std::cout << p << std::endl;
	}

	//////////////////////
	// Compute clusters //
	//////////////////////
	//std::sort(centroids.begin(), centroids.end(), poseCluster::compare);

	// Normalize centroids
	float totalModelVotes=static_cast<float>(model->modelCloud->size()*(model->modelCloud->size()-1));
	//std::cout << totalModelVotes << " " << model->totalSurfaceArea << std::endl;
	//std::cout << "Best centroid score before: " << centroids[0].votes << std::endl;


	//std::cout << "Best centroid score after: " << centroids[0].votes << std::endl;
	// end normalize centroids
	//std::cout << std::endl << "Number of poses:" <<bestPoses.size() << std::endl << std::endl;
	//std::cout << std::endl << "Best pose votes:" <<bestPoses[0].votes << std::endl << std::endl;
	if(filterOn)
	{

        std::cout << "Best centroid score: " << centroids[0].votes << std::endl;
        std::cout << "Best centroid score(normalized): " << (float) centroids[0].votes/totalModelVotes << std::endl;
		clusters.reserve(centroids.size());
		for(size_t c=0; c < centroids.size(); ++c)
		//for(size_t c=0; c < 1; ++c)
		{

			clusters.push_back(cluster(pose (centroids[c].votes, transformation(centroids[c].rotation(),centroids[c].translation() ) ),centroids[c].poses ) );
		}
		clusterClusters(clusters);
		std::sort(clusters.begin(), clusters.end(), cluster::compare);
	}
	else
	{
		std::sort(centroids.begin(), centroids.end(), poseCluster::compare);
		//clusters.push_back(cluster(pose (centroids[0].votes, transformation(centroids[0].rotation(),centroids[0].translation() ) ),centroids[0].poses )); //METER
		for(size_t c=0; c < centroids.size(); ++c) //TIRAR DEPOIS DOS TESTS
		//for(size_t c=0; c < 1; ++c)
		{

			clusters.push_back(cluster(pose (centroids[c].votes, transformation(centroids[c].rotation(),centroids[c].translation() ) ),centroids[c].poses ) );
		}
	}

	for(size_t c=0; c < clusters.size(); ++c)
	{
		clusters[c].normalizeVotes(model->totalSurfaceArea/totalModelVotes); // normalize votes weighs by the argument factor
		//std::cout << "centroid poses:" << centroids[c].poses.size()<< "centroid score after: " << centroids[c].votes << std::endl;
	}


	/*for(size_t p=0; p< 1; ++p)
	{
		Eigen::Vector3f eulerAngles;
		objectModel::quaternionToEuler(clusters[p].meanPose.transform.rotation, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		float x,y,z;
		std::cout << "1. EULER ANGLES: "<< std::endl << eulerAngles.transpose()*RAD_TO_DEG << std::endl;
		pcl::getTranslationAndEulerAngles (clusters[p].meanPose.getTransformation(), x, y, z, eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		std::cout << "2. EULER ANGLES: "<< std::endl << eulerAngles.transpose()*RAD_TO_DEG << std::endl << std::endl;

	}*/

	return clusters;
}

// Cluster the existent clusters into well defined bins method (needed by grid-based filters)
void poseEstimationSV::clusterClusters(std::vector<cluster> & clusters)
{
	//std::cout << "clusters before:" << clusters.size() << std::endl;
	bool newMerge=true;
	int it=0;
 	while(newMerge)
	{
		//std::cout << "cycle number: " << ++it << std::endl;
		newMerge=false;
		for(std::vector<cluster>::iterator c1=clusters.begin(); c1 < clusters.end(); ++c1)
		{
			for(std::vector<cluster>::iterator c2=c1+1; c2 < clusters.end();)
			{
					if( ( orientationDistance(*c1,*c2) >= pointPair::angleStepCos ) && ( positionDistance(*c1, *c2)) <= (model->maxModelDistSquared*0.05*0.05 ) )
					{
						if(isnan(acos(orientationDistance(*c1, *c2))))
						{
							std::cout << c1->meanPose.transform.rotation.w() << " " << c1->meanPose.transform.rotation.x() << " " << c1->meanPose.transform.rotation.y() << " " << c1->meanPose.transform.rotation.z() << std::endl;
							std::cout << c2->meanPose.transform.rotation.w() << " " << c2->meanPose.transform.rotation.x() << " " << c2->meanPose.transform.rotation.y() << " " << c2->meanPose.transform.rotation.z() << std::endl;
							std::cout << orientationDistance(*c1, *c2) << std::endl;
							exit(1);
						}
						if(positionDistance(*c1, *c2) <= (model->maxModelDistSquared*0.05*0.05 ))
						{
							//  average orientation
							Eigen::Quaternion<float> q((c1->meanPose.transform.rotation.w()*c1->meanPose.votes)+(c2->meanPose.transform.rotation.w()*c2->meanPose.votes),
							(c1->meanPose.transform.rotation.x()*c1->meanPose.votes)+(c2->meanPose.transform.rotation.x()*c2->meanPose.votes),
							(c1->meanPose.transform.rotation.y()*c1->meanPose.votes)+(c2->meanPose.transform.rotation.y()*c2->meanPose.votes),
							(c1->meanPose.transform.rotation.z()*c1->meanPose.votes)+(c2->meanPose.transform.rotation.z()*c2->meanPose.votes));
							// normalize quaternion
							q.normalize();

							c1->meanPose.transform.rotation=q;

							// average position
							float votationSumInv=1/(c1->meanPose.votes+c2->meanPose.votes);
							Eigen::Translation3f transAux;
							transAux.x()=( (c1->meanPose.transform.translation.x()*c1->meanPose.votes) + (c2->meanPose.transform.translation.x()*c2->meanPose.votes) )*votationSumInv;
							transAux.y()=( (c1->meanPose.transform.translation.y()*c1->meanPose.votes) + (c2->meanPose.transform.translation.y()*c2->meanPose.votes) )*votationSumInv;
							transAux.z()=( (c1->meanPose.transform.translation.z()*c1->meanPose.votes) + (c2->meanPose.transform.translation.z()*c2->meanPose.votes) )*votationSumInv;
							/*transAux.x()=( c1->meanPose.transform.translation.x() + c2->meanPose.transform.translation.x() )*0.5;
							transAux.y()=( c1->meanPose.transform.translation.y() + c2->meanPose.transform.translation.y() )*0.5;
							transAux.z()=( c1->meanPose.transform.translation.z() + c2->meanPose.transform.translation.z() )*0.5;*/
							//std::cout << "VERRR ISTO:" << positionDistance(*c1, *c2) << " "<<acos(orientationDistance(*c1,*c2))*RAD_TO_DEG << std::endl;
							//std::cout << "TRANS BEFORE:" << c1->meanPose.transform.translation.x() << " " << c1->meanPose.transform.translation.y()  << " " << c1->meanPose.transform.translation.z() << std::endl;
							c1->meanPose.transform.translation=transAux;
							//std::cout << "TRANS AFTER:" << c1->meanPose.transform.translation.x() << " " << c1->meanPose.transform.translation.y()  << " " << c1->meanPose.transform.translation.z() << std::endl;
							c1->meanPose.votes+=c2->meanPose.votes;


						}
						// remove cluster 2	
						c2=clusters.erase(c2);
						newMerge=true;
					}
					else
						++c2;
			}
		}

	}
	//std::cout << "clusters after:" << clusters.size() << std::endl;
}
