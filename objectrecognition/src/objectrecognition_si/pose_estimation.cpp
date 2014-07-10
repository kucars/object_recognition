#include "objectrecognition_si/pose_estimation.h"

// Static variables
unsigned int poseEstimationSI::scaleBinOffset;
unsigned int poseEstimationSI::scaleBins;
unsigned int poseEstimationSI::scaleMaxBin;
float poseEstimationSI::logBase;
float poseEstimationSI::logBaseLog;
float poseEstimationSI::logBaseLogInverted;
//float poseEstimationSI::scaleLowerBound;

clock_t timeIni;
double timeEnd;
void Tic(){ timeIni=clock(); }
void Tac(){ timeEnd = (double)(clock()-timeIni)/(double)CLOCKS_PER_SEC; std::cout << timeEnd*1000 << "ms (" << 1/timeEnd << " fps)" << std::endl;}

poseEstimationSI::poseEstimationSI(objectModelPtr inputModel)
{	
	///////////////////////////
   	// Initialize parameters //
	///////////////////////////

	model=inputModel;

	////////////////////////////////////////////////////////
	// Allocate space for several accumulators (parallel) //
	////////////////////////////////////////////////////////
	accumulatorParallelAux.resize(4);
	for(int i=0;i < omp_get_num_procs(); ++i)
	{
		// Allocate space for reference points (lines)
		accumulatorParallelAux[i].resize(model->modelCloud->size());
		for(std::vector< std::vector<std::vector<int> > >::iterator refIt=accumulatorParallelAux[i].begin();refIt < accumulatorParallelAux[i].end(); ++refIt)
		{
			// Allocate space for poses (columns)
			refIt->resize(pointPair::angleBins);
			for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
			{
				// Allocate space for scales
				angleIt->resize(scaleBins);
			}
		}
	}

	////////////////////////////////////////
	// Allocate space for the accumulator //
	////////////////////////////////////////

	// Allocate space for reference points
	accumulator.resize(model->modelCloud->size());
	for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
	{
		// Allocate space for poses
		refIt->resize(pointPair::angleBins);
		for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
		{
			// Allocate space for scales
			angleIt->resize(scaleBins);
		}
	}
}

poseEstimationSI::~poseEstimationSI()
{	
	///////////////////////////////////////////
	// Deallocate space for the accumulators //
	///////////////////////////////////////////

	for(int i=0;i < omp_get_num_procs(); ++i)
	{
		for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulatorParallelAux[i].begin();refIt < accumulatorParallelAux[i].end(); ++refIt)
		{
			for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
			{
				// Deallocate space for scales
				angleIt->clear();
			}
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

	for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
	{

		for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
		{
			angleIt->clear();
		}
		// Deallocate space allocated for poses
		refIt->clear();
	}
	// Deallocate space allocated for reference points
	accumulator.clear();
}

// METHOD THAT RECEIVES POINT CLOUDS
std::vector<poseEstimationSI::clusterPtr> poseEstimationSI::poseEstimationCore(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::vector < posePtr > bestPoses;

	int bestPoseAlpha;
	int bestPosePoint;
	int bestPoseScale;
	int bestPoseVotes;

	pcl::PassThrough<pcl::PointXYZ> nan_remove;
	pcl::PointIndices normals_nan_indices;
	pcl::ExtractIndices<pcl::PointNormal> nan_extract;

	float alpha, scale;
	unsigned int alphaBin, index;
	int scaleBin;
	// Iterators
	std::vector<int>::iterator sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSI>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;

	std::cout<< "\tCloud size: " << cloud->size() << endl;
	
	//////////////////////////////////////////////
	// Downsample point cloud using a voxelgrid //
	//////////////////////////////////////////////
	cloudDownsampled=filtering::downsampleXYZ(cloud,model->distanceStep);
	std::cout<< "\tCloud size after downsampling: " << cloudDownsampled->size() << endl;


	// Compute point cloud normals (using cloud before downsampling information)
	std::cout<< "\tCompute normals... ";
	cloudNormals=model->computeNormals(cloudDownsampled);
	std::cout<< "Done" << endl;

	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i) 
	{
		if (isnan(cloudNormals->points[i].normal[0]))
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
	// VOTATION //
	//////////////

	std::cout<< "\tVotation... ";

	for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		Eigen::Vector3f scenePoint=cloudWithNormalsDownSampled->points[*sr].getVector3fMap();
		Eigen::Vector3f sceneNormal=cloudWithNormalsDownSampled->points[*sr].getNormalVector3fMap ();

		// Get transformation from scene frame to global frame
		Eigen::AngleAxisf rotationSceneToGlobal (acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized());
		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
		{
			for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
				std::fill(angleIt->begin(),angleIt->end(),0); 
		}
		
		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[*sr].x==si->x) && (cloudWithNormalsDownSampled->points[*sr].y==si->y) && (cloudWithNormalsDownSampled->points[*sr].z==si->z))
			{
				continue;
			}	

			// Compute SIPPF
			pointPairSI SIPPF=pointPairSI(cloudWithNormalsDownSampled->points[*sr],*si);

			// Compute index
			index=SIPPF.getHash(*si);

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue;

			// Iterate over similar point pairs
			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{

				scale=feature[0]*sameFeatureIt->distanceInverted;
				scaleBin=round(log(scale)*logBaseLogInverted)+scaleBinOffset;

				if(scaleBin>(int)scaleMaxBin||scaleBin<0)
					continue;
				//std::cout << "scale:" << scale << " scale bin:" << scaleBin <<" scale max bin:" << (int)scaleMaxBin<<std::endl;

				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-SIPPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;

				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication
				if(alphaBin==pointPair::angleBins) alphaBin=0;

				++accumulator[sameFeatureIt->id][alphaBin][scaleBin];
			}
		}

		// Choose best pose (highest peak on the accumulator[peak with more votes])
		bestPosePoint=0;
		bestPoseAlpha=0;
		bestPoseScale=0;
		bestPoseVotes=0;
		
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulator[p][a][s]>bestPoseVotes)
					{
						bestPoseVotes=accumulator[p][a][s];
						bestPosePoint=p;
						bestPoseAlpha=a;
						bestPoseScale=s;
					}	
				}
			}
		}	

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			//std::cout <<logBase << " "<<scaleBinOffset<< " "<< bestPoseScale<< " " << static_cast<float>(bestPoseScale-scaleBinOffset) << " "<< pow(logBase,static_cast<float>(bestPoseScale-scaleBinOffset)) << std::endl;
			// Compute and store transformation from model to scene
			boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(model->modelCloud->points[bestPosePoint],transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI,pow(logBase,bestPoseScale-scaleBinOffset)) ) );
			bestPoses.push_back(bestPose);
		}
		else 
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulator[bestPosePoint][bestPoseAlpha][bestPoseScale]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulator[p][a][s]>=accumulatorPeakThreshold*bestPoseVotes)
					{
						// Compute and store transformation from model to scene
						boost::shared_ptr<pose> bestPose(new pose( accumulator[p][a][s],model->modelToScene(model->modelCloud->points[p],transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI,pow(logBase,s-scaleBinOffset)) ) );
						bestPoses.push_back(bestPose);
					}
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
	std::cout << "\tCompute clusters... ";
	clusters=poseClustering(bestPoses);
	std::cout << "Done" << std::endl;

	return clusters;
}

// METHOD THAT RECEIVES POINT CLOUDS (OPEN MP)
std::vector<poseEstimationSI::clusterPtr> poseEstimationSI::poseEstimationCore_openmp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::vector < posePtr > bestPoses;
	std::vector <std::vector < posePtr > > bestPosesAux;
	bestPosesAux.resize(omp_get_num_procs());

	int bestPoseAlpha;
	int bestPosePoint;
	int bestPoseScale;
	int bestPoseVotes;

	pcl::PassThrough<pcl::PointXYZ> nan_remove;
	pcl::PointIndices normals_nan_indices;
	pcl::ExtractIndices<pcl::PointNormal> nan_extract;

	float alpha, scale;
	unsigned int alphaBin, index;
	int scaleBin;
	// Iterators
	std::vector<int>::iterator sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSI>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;

	std::cout<< "\tCloud size: " << cloud->size() << endl;

	//////////////////////////////////////////////
	// Downsample point cloud using a voxelgrid //
	//////////////////////////////////////////////
	cloudDownsampled=filtering::downsampleXYZ(cloud,model->distanceStep);
	std::cout<< "\tCloud size after downsampling: " << cloudDownsampled->size() << endl;


	// Compute point cloud normals (using cloud before downsampling information)
	std::cout<< "\tCompute normals... ";
	cloudNormals=model->computeNormals(cloudDownsampled);
	std::cout<< "Done" << endl;

	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i)
	{
		if (isnan(cloudNormals->points[i].normal[0]))
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
	// VOTATION //
	//////////////

	std::cout<< "\tVotation... ";

	omp_set_num_threads(omp_get_num_procs());

	#pragma omp parallel for private(alpha,alphaBin,sameFeatureIt,index,feature,si,_pointTwoTransformed,scale,scaleBin) //reduction(+:iteration)  //nowait
	for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		Eigen::Vector3f scenePoint=cloudWithNormalsDownSampled->points[*sr].getVector3fMap();
		Eigen::Vector3f sceneNormal=cloudWithNormalsDownSampled->points[*sr].getNormalVector3fMap ();

		// Get transformation from scene frame to global frame
		Eigen::AngleAxisf rotationSceneToGlobal (acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized());
		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
		{
			for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
				std::fill(angleIt->begin(),angleIt->end(),0);
		}

		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[*sr].x==si->x) && (cloudWithNormalsDownSampled->points[*sr].y==si->y) && (cloudWithNormalsDownSampled->points[*sr].z==si->z))
			{
				continue;
			}

			// Compute SIPPF
			pointPairSI SIPPF=pointPairSI(cloudWithNormalsDownSampled->points[*sr],*si);

			// Compute index
			index=SIPPF.getHash(*si);

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue;

			// Iterate over similar point pairs
			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{

				scale=feature[0]*sameFeatureIt->distanceInverted;
				scaleBin=round(log(scale)*logBaseLogInverted)+scaleBinOffset;

				if(scaleBin>(int)scaleMaxBin||scaleBin<0)
					continue;
				//std::cout << "scale:" << scale << " scale bin:" << scaleBin <<" scale max bin:" << (int)scaleMaxBin<<std::endl;

				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-SIPPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;

				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication
				if(alphaBin==pointPair::angleBins) alphaBin=0;

				++accumulatorParallelAux[omp_get_thread_num()][sameFeatureIt->id][alphaBin][scaleBin];
			}
		}

		// Choose best pose (highest peak on the accumulator[peak with more votes])
		bestPosePoint=0;
		bestPoseAlpha=0;
		bestPoseScale=0;
		bestPoseVotes=0;

		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulatorParallelAux[omp_get_thread_num()][p][a][s]>bestPoseVotes)
					{
						bestPoseVotes=accumulatorParallelAux[omp_get_thread_num()][p][a][s];
						bestPosePoint=p;
						bestPoseAlpha=a;
						bestPoseScale=s;
					}
				}
			}
		}

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			//std::cout <<logBase << " "<<scaleBinOffset<< " "<< bestPoseScale<< " " << static_cast<float>(bestPoseScale-scaleBinOffset) << " "<< pow(logBase,static_cast<float>(bestPoseScale-scaleBinOffset)) << std::endl;
			// Compute and store transformation from model to scene
			boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(model->modelCloud->points[bestPosePoint],transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI,pow(logBase,bestPoseScale-scaleBinOffset)) ) );
			bestPoses.push_back(bestPose);
		}
		else
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulatorParallelAux[omp_get_thread_num()][bestPosePoint][bestPoseAlpha][bestPoseScale]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulatorParallelAux[omp_get_thread_num()][p][a][s]>=accumulatorPeakThreshold*bestPoseVotes)
					{
						// Compute and store transformation from model to scene
						boost::shared_ptr<pose> bestPose(new pose( accumulatorParallelAux[omp_get_thread_num()][p][a][s],model->modelToScene(model->modelCloud->points[p],transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI,pow(logBase,s-scaleBinOffset)) ) );
						bestPoses.push_back(bestPose);
					}
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
	std::cout << "\tCompute clusters... ";
	clusters=poseClustering(bestPoses);
	std::cout << "Done" << std::endl;

	return clusters;
}

// METHOD THAT RECEIVES SURFLET CLOUDS
std::vector<poseEstimationSI::clusterPtr> poseEstimationSI::poseEstimationCore(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	std::vector < posePtr > bestPoses;

	int bestPoseAlpha;
	int bestPosePoint;
	int bestPoseScale;
	int bestPoseVotes;

	pcl::PassThrough<pcl::PointXYZ> nan_remove;
	pcl::PointIndices normals_nan_indices;
	pcl::ExtractIndices<pcl::PointNormal> nan_extract;

	float alpha, scale;
	unsigned int alphaBin, index;
	int scaleBin;
	// Iterators
	std::vector<int>::iterator sr; // scene reference point
	pcl::PointCloud<pcl::PointNormal>::iterator si;	// scene paired point
	std::vector<pointPairSI>::iterator sameFeatureIt; // same key on hash table
	std::vector<boost::shared_ptr<pose> >::iterator bestPosesIt;

	Eigen::Vector4f feature;

	std::cout << "\tDownsample dense surflet cloud... " << std::endl;
	cout << "\t\tSurflet cloud size before downsampling: " << cloud->size() << endl;
 	// Create the filtering object
  	pcl::VoxelGrid<pcl::PointNormal> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (model->distanceStep,model->distanceStep,model->distanceStep);
  	sor.filter (*cloudNormals);
	cout << "\t\tSurflet cloud size after downsampling: " << cloudNormals->size() << endl;
	std::cout<< "\tDone" << std::endl;

	//////////////////////////////////////////////////////////////////////////////
	// Filter again to remove spurious normals nans (and it's associated point) //
	//////////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < cloudNormals->points.size(); ++i)
	{
		if (isnan(cloudNormals->points[i].normal[0]))
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
	// VOTATION //
	//////////////

	std::cout<< "\tVotation... ";

	for(sr=referencePointsIndices->indices.begin(); sr < referencePointsIndices->indices.end(); ++sr)
	{
		Eigen::Vector3f scenePoint=cloudWithNormalsDownSampled->points[*sr].getVector3fMap();
		Eigen::Vector3f sceneNormal=cloudWithNormalsDownSampled->points[*sr].getNormalVector3fMap ();

		// Get transformation from scene frame to global frame
		Eigen::AngleAxisf rotationSceneToGlobal (acosf (sceneNormal.dot (Eigen::Vector3f::UnitX ())),sceneNormal.cross (Eigen::Vector3f::UnitX ()). normalized());
		Eigen::Affine3f transformSceneToGlobal = Eigen::Translation3f ( rotationSceneToGlobal* ((-1)*scenePoint)) * rotationSceneToGlobal;

		//////////////////////
		// Choose best pose //
		//////////////////////

		// Reset pose accumulator
		for(std::vector<std::vector<std::vector<int> > >::iterator refIt=accumulator.begin();refIt < accumulator.end(); ++refIt)
		{
			for(std::vector<std::vector<int> >::iterator angleIt=refIt->begin();angleIt < refIt->end(); ++angleIt)
				std::fill(angleIt->begin(),angleIt->end(),0);
		}

		for(si=cloudWithNormalsDownSampled->begin(); si < cloudWithNormalsDownSampled->end();++si)
		{
			// if same point, skip point pair
			if( (cloudWithNormalsDownSampled->points[*sr].x==si->x) && (cloudWithNormalsDownSampled->points[*sr].y==si->y) && (cloudWithNormalsDownSampled->points[*sr].z==si->z))
			{
				continue;
			}

			// Compute SIPPF
			pointPairSI SIPPF=pointPairSI(cloudWithNormalsDownSampled->points[*sr],*si);

			// Compute index
			index=SIPPF.getHash(*si);

			// If there is no similar point pair features in the model, skip point pair and avoid computing the alpha
			if(model->hashTable[index].size()==0)
				continue;

			// Iterate over similar point pairs
			for(sameFeatureIt=model->hashTable[index].begin(); sameFeatureIt<model->hashTable[index].end(); ++sameFeatureIt)
			{

				scale=feature[0]*sameFeatureIt->distanceInverted;
				scaleBin=round(log(scale)*logBaseLogInverted)+scaleBinOffset;

				if(scaleBin>(int)scaleMaxBin||scaleBin<0)
					continue;
				//std::cout << "scale:" << scale << " scale bin:" << scaleBin <<" scale max bin:" << (int)scaleMaxBin<<std::endl;

				// Vote on the reference point and angle (and object)
				alpha=sameFeatureIt->alpha-SIPPF.alpha; // alpha values between [-360,360]

				// alpha values should be between [-180,180] ANGLE_MAX = 2*PI
				if(alpha<(-PI))
					alpha=ANGLE_MAX+alpha;
				else if(alpha>(PI))
					alpha=alpha-ANGLE_MAX;

				alphaBin=static_cast<unsigned int> ( round((alpha+PI)*pointPair::angleStepInverted) ); // division is slower than multiplication
				if(alphaBin==pointPair::angleBins) alphaBin=0;

				++accumulator[sameFeatureIt->id][alphaBin][scaleBin];
			}
		}

		// Choose best pose (highest peak on the accumulator[peak with more votes])
		bestPosePoint=0;
		bestPoseAlpha=0;
		bestPoseScale=0;
		bestPoseVotes=0;

		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulator[p][a][s]>bestPoseVotes)
					{
						bestPoseVotes=accumulator[p][a][s];
						bestPosePoint=p;
						bestPoseAlpha=a;
						bestPoseScale=s;
					}
				}
			}
		}

		// A candidate pose was found
		if(bestPoseVotes!=0)
		{
			//std::cout <<logBase << " "<<scaleBinOffset<< " "<< bestPoseScale<< " " << static_cast<float>(bestPoseScale-scaleBinOffset) << " "<< pow(logBase,static_cast<float>(bestPoseScale-scaleBinOffset)) << std::endl;
			// Compute and store transformation from model to scene
			boost::shared_ptr<pose> bestPose(new pose( bestPoseVotes,model->modelToScene(model->modelCloud->points[bestPosePoint],transformSceneToGlobal,static_cast<float>(bestPoseAlpha)*pointPair::angleStep-PI,pow(logBase,bestPoseScale-scaleBinOffset)) ) );
			bestPoses.push_back(bestPose);
		}
		else
		{
			continue;
		}

		// Choose poses whose votes are a percentage above a given threshold of the best pose
		accumulator[bestPosePoint][bestPoseAlpha][bestPoseScale]=0; 	// This is more efficient than having an if condition to verify if we are considering the best pose again
		for(size_t p=0; p < model->modelCloud->size(); ++p)
		{
			for(unsigned int a=0; a < pointPair::angleBins; ++a)
			{
				for(unsigned int s=0; s < scaleBins; ++s)
				{
					if(accumulator[p][a][s]>=accumulatorPeakThreshold*bestPoseVotes)
					{
						// Compute and store transformation from model to scene
						boost::shared_ptr<pose> bestPose(new pose( accumulator[p][a][s],model->modelToScene(model->modelCloud->points[p],transformSceneToGlobal,static_cast<float>(a)*pointPair::angleStep-PI,pow(logBase,s-scaleBinOffset)) ) );
						bestPoses.push_back(bestPose);
					}
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
	std::cout << "\tCompute clusters... ";
	clusters=poseClustering(bestPoses);
	std::cout << "Done" << std::endl;

	return clusters;
}



// CLUSTERING METHOD
std::vector<poseEstimationSI::clusterPtr> poseEstimationSI::poseClustering(std::vector < posePtr > & bestPoses)
{
	// Sort clusters
	std::sort(bestPoses.begin(), bestPoses.end(), pose::compare);

	std::vector< boost::shared_ptr<poseCluster> > centroids;
	std::vector< boost::shared_ptr<cluster> > transformations;    

	float _orientationDistance;	
	float _positionDistance;
	float _scaleDistance;

	bool found;
	
	int nearestCentroid;

	///////////////////////////////
	// Initialize first centroid //
	///////////////////////////////

	// If there are no poses, return
	if(bestPoses.empty())
		return transformations;


	centroids.push_back(boost::shared_ptr<poseCluster> (new poseCluster(0,bestPoses[0])));
	//centroids.back()->poses.push_back(bestPoses[0]);

	Tic();	
	// For each pose
	for(size_t p=1; p< bestPoses.size(); ++p)
	{
		found=false;
		for(size_t c=0; c < centroids.size(); ++c)
		{
			//std::cout << std::endl << "\tcheck if centroid:" << std::endl;
			// Compute (squared) distance to the cluster center
			_orientationDistance=orientationDistance(bestPoses[p],bestPoses[centroids[c]->poseIndex]);
			_positionDistance=positionDistance(bestPoses[p],bestPoses[centroids[c]->poseIndex]);
			_scaleDistance=scaleDistance(bestPoses[p],bestPoses[centroids[c]->poseIndex]);
			// If the cluster is nearer than a threshold add current pose to the cluster
			//if((_orientationDistance>=poseAngleStepCos) && _positionDistance<=((model->maxModelDistSquared)*0.05*0.05))
			//if((_orientationDistance>=cos(acos(poseAngleStepCos)) ) && _positionDistance<=((model->maxModelDistSquared)*0.1*0.1))
			if((_orientationDistance>=pointPair::angleStepCos) && (_positionDistance<=model->halfDistanceStepSquared) &&(equalFloat(_scaleDistance,0.0))) // VER ISTO
			{

				nearestCentroid=c;
				found=true;
				break;
			}		
		}			
			
		if(found)
		{
			centroids[nearestCentroid]->update(bestPoses[p]);
		}
		else
		{
			boost::shared_ptr<poseCluster> clus(new poseCluster(p,bestPoses[p]));
			centroids.push_back(clus);
		}
	}
	Tac();	

	//////////////////////
	// Compute clusters //
	//////////////////////
	std::sort(centroids.begin(), centroids.end(), poseCluster::compare);
	//std::cout << std::endl << "Number of poses:" <<bestPoses.size() << std::endl << std::endl;
	//std::cout << std::endl << "Best pose votes:" <<bestPoses[0]->votes << std::endl << std::endl;
	//for(size_t c=0; c < centroids.size(); ++c)
	if(filterOn)
	{
		for(size_t c=0; c < centroids.size(); ++c)
		//for(size_t c=0; c < 1; ++c)
		{
			//std::cout << centroids[c]->votes << std::endl;
			transformations.push_back(boost::shared_ptr<cluster> (new cluster( boost::shared_ptr<pose> (new pose (centroids[c]->votes, boost::shared_ptr<transformation>( new transformation(centroids[c]->rotation(),centroids[c]->translation(),centroids[c]->scaling() ) ) ) ),centroids[c]->poses ) ) );
		}
	}
	else
	{
		transformations.push_back(boost::shared_ptr<cluster> (new cluster( boost::shared_ptr<pose> (new pose (centroids[0]->votes, boost::shared_ptr<transformation>( new transformation(centroids[0]->rotation(),centroids[0]->translation(),centroids[0]->scaling() ) ) ) ),centroids[0]->poses ) ) );
	}

	return transformations;
}

float poseEstimationSI::scaleDistance(poseEstimation::posePtr bestPose, poseEstimation::posePtr bestCentroid)
{
	// Compute distance between scalings
	return abs(bestPose->transform->scale.factor() - bestCentroid->transform->scale.factor());
}


