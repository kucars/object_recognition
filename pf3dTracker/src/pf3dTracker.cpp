//TODO 2012
// Get rid of the hardcoding of the position of initialization files
// Fix time measurement operations


/**
*
* Library of the 3d position tracker implementing the particle filter.
* See \ref icub_pf3dtracker \endref
*
* Copyright (C) 2009 RobotCub Consortium
*
* Author: Matteo Taiana
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

//
//
//
// NOTES:
// distances are trated as in millimeters inside the program, but they are converted to meters when communicating to the outside.
// likelihood is normalized only when communicated to the outside.

// TODO
//
//  0. make things compile and build.
//
//  1. test that the function that computes the histogram from the RGB image works.
//
//  1. check that things behave properly
//
//  2. make sure randomization is really random.
//
//  3. remove stuff that is not used
//
//  4. make the code check the return values (ROBUSTNESS/DEBUGGABILITY).
//
//  5. try to optimize the code, make it faster.
//

//  X. fix the memory leak. DONE

//          il RESAMPLING HA DEI PROBLEMI???: tipicamente la matrice delle particelle e' piena di n-a-n dopo il resampling, se uso poche particelle (non ho provato con molte a vedere come e' l'output).
//          con 5000 particelle sembra che funzioni tutto...

//things to be done:

//1. when the ball is moving towards the camera, the tracker lags behind it... is this because particles that are nearer than the true value to the camera have zero likelihood while the ones that are farther have higher likelihood values? how could i test this? plotting the positions of the particles? how could i solve this?

//2. check the resampling function: sometimes when the image goes blank the tracker programs quits suddenly. it's probably that function trying to write a particle value in an area where it should not (array overflow).

// measure which parts of the tracking loop are using up processing time.

//WHISHLIST:
//turn all static arrays into pointers to pointers, so that their size can be chosen at run time (through the configuration file)
//remove unnecessary included headers and namespaces.
//comment the whole project in doxygen style.
//create a pair of support files (.hpp and .cpp) specifically for this project.
//create a variable to contain the parameters for the motion model
//the size of the image is not a parameter.
//there are some functions which have among their parameters "inside_outside_...". this should be fixed, as this value is now defined at compile time.

//DONE:
//fixed: memory leak. when is the memory allocated by OpenCV freed? never? take a look at the small functions I call repeatedly: they might be the cause for the memory leak. use: cvReleaseImage.
//the images are visualized through opencv: change this to the output video port.
//the output data port is not used so far: it should stream the numeric values of the estimates.
//fixed the seg-fault problem. s-f happen when stdDev is high (or at the beginning of a tracking)... seems like a problem in the resampling algorithm.
//fixed. it was a prolem in the randomization that was not really random. 1. check the resampling algorithm: when the acceleration error is low, the tracker starts with particles in 0,0,0 and stays there.
//maybe this is due to the fact that the first images the tracker receives are meaningless, the likelihood is quite low, so the particles are spread a lot and one happens to be near the origin. all the particles are concentrated there and if you don't have a high acceleration noise you get stuck there.
//done. write particles status/likelihood on the particleOutputPort, for visualization purposes.
//   write a client that reads this data and plots particles.

#include <iCub/pf3dTracker.hpp>

//#include <highgui.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <time.h>

//m#include <gsl/gsl_math.h>
//#include <pf3dTracker/estimates.h>
clock_t time1;
double time2;
double fps;
void tic(){ time1=clock(); }
double tac(){ return (double)(clock()-time1)/(double)CLOCKS_PER_SEC;}


void printMat(cv::Mat & A);


//constructor
PF3DTracker::PF3DTracker(ros::NodeHandle n): n_(n), it_(n), n_priv("~") 
{
    _staticImageTest = false;

    bool setUpDoneCorrectly = false;
    // Read initialization variables and set the tracker up
    setUpDoneCorrectly = open();
    if(!setUpDoneCorrectly)
    {
        cout<<"Set up of pf3dTracker failed."<<endl;
        ros::shutdown();
    }

}



//member function that set the object up.
bool PF3DTracker::open()
{
    cout<<"Starting pf3dTracker"<<endl;
    _doneInitializing=false; //TODO this can probably be removed

    bool failure;
    bool quit;
    string trackedObjectColorTemplate;
    string dataFileName;
    string trackedObjectShapeTemplate;
    string motionModelMatrix;
    string temp;
    int row, column;

    quit=false;

    _lut = new Lut[256*256*256];


    // Set some parameters that are no longer used and should be removed TODO
    _colorTransfPolicy=1;

    
    
    //***********************************************
    // Read parameters from the initialization file *
    //***********************************************
    //Topics
    n_priv.param<std::string>("inputVideoPort",  _inputVideoPortName,  "/camera/rgb/image_color");
    n_priv.param<std::string>("outputVideoPort", _outputVideoPortName, "/pf3dTracker/video_o");
    n_priv.param<std::string>("outputDataPort",  _outputDataPortName,  "/pf3dTracker/data_o");
    
    //Parameters for the algorithm
    n_priv.param<int>("nParticles", _nParticles, 900); //TODO set it back to 900
    n_priv.param<double>("accelStDev", _accelStDev, 30.0); //TODO set it back to 30
    n_priv.param<double>("insideOutsideDiffWeight", _insideOutsideDifferenceWeight, 1.5);
    n_priv.param<double>("likelihoodThreshold", _likelihoodThreshold, 0.005);//.005); TODO ser it back to 0.005

    n_priv.param<std::string>("trackedObjectColorTemplate", trackedObjectColorTemplate, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/red_ball_iit.bmp");
    n_priv.param<std::string>("trackedObjectShapeTemplate", trackedObjectShapeTemplate, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/initial_ball_points_smiley_31mm_20percent.csv");
    n_priv.param<std::string>("motionModelMatrix", motionModelMatrix, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/motion_model_matrix.csv");
    n_priv.param<std::string>("trackedObjectTemp", dataFileName, "current_histogram.csv"); //might be removed? TODO
    n_priv.param<std::string>("initializationMethod", _initializationMethod, "3dEstimate");
    n_priv.param<double>("initialX", _initialX, 0.0);
    n_priv.param<double>("initialY", _initialY, 0.0);
    n_priv.param<double>("initialZ", _initialZ, 0.2);
    //Units are meters outside the program, but millimeters inside, so we need to convert
    _initialX*=1000.0;
    _initialY*=1000.0;
    _initialZ*=1000.0;
    
    //Camera intrinsic parameters
    n_priv.param<int>("w", _calibrationImageWidth,        320); //NOT USED AT THE MOMENT
    n_priv.param<int>("h", _calibrationImageHeight,       240);  //NOT USED AT THE MOMENT
    n_priv.param<double>("perspectiveFx", _perspectiveFx, 980.0);
    n_priv.param<double>("perspectiveFy", _perspectiveFy, 982.0);
    n_priv.param<double>("perspectiveCx", _perspectiveCx, 320.0);
    n_.param<double>("perspectiveCy", _perspectiveCy, 240.0);


    std::cout << "initialX: " << _initialX << std::endl;
    std::cout << "initialY: " << _initialY << std::endl;
    std::cout << "initialZ: " << _initialZ << std::endl;
    if(_staticImageTest)
    {
        _nParticles=1;
        _accelStDev=0.0001;
        _initialX =  -45.0; //careful: these are millimeters!
        _initialY =  -45.0; //careful: these are millimeters!
        _initialZ =  290.0; //careful: these are millimeters!
    }
    
    //*********************************************
    // Create and initialize some data structures *
    //*********************************************
    //Create the color transformation Look Up Table
    //I could write the lut on a file and read it back, I don't know if that would be faster
    fillLut(_lut);

    //make sure random numbers really are random.
    srand((unsigned int)time(0));
    rngState = cvRNG(rand());

    //allocate some memory and initialize some data structures for colour histograms.
    int dimensions;
    dimensions=3;
    int sizes[3]={YBins,UBins,VBins};
    _modelHistogramMat=cvCreateMatND(dimensions, sizes, CV_64FC1);
    if(_modelHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _modelHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }
    _innerHistogramMat=cvCreateMatND(dimensions, sizes, CV_64FC1);
    if(_innerHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _innerHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }
    _outerHistogramMat=cvCreateMatND(dimensions, sizes, CV_64FC1);
    if(_outerHistogramMat==0)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _outerHistogramMat.\n";
        fflush(stdout);
        quit =true;
    }

    _model3dPointsMat=cv::Mat(3, 2*nPixels, CV_64FC1);
    if(!_model3dPointsMat.data)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _model3dPointsMat.\n";
        fflush(stdout);
        quit =true;
    }
    _points2Mat=cv::Mat(3, 2*nPixels, CV_64FC1);
    if(!_points2Mat.data)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _points2Mat.\n";
        fflush(stdout);
        quit =true;
    }
    _tempMat=cv::Mat(3, 2*nPixels, CV_64FC1);
    if(!_tempMat.data)
    {
        cout<<"PF3DTracker::open - I wasn\'t able to allocate memory for _tempMat.\n";
        fflush(stdout);
        quit =true;
    }




    //TODO merge these two functions
    //Build and read the color model for the tracked object
    failure=computeTemplateHistogram(trackedObjectColorTemplate,dataFileName);
    if(failure)
    {
        cout<<"I had troubles computing the template histogram."<<endl;
        quit=true;
    }
    failure=readModelHistogram(_modelHistogramMat,dataFileName.c_str());
    if(failure)
    {
        cout<<"I had troubles reading the template histogram."<<endl;
        quit=true;
    }

    //Read the shape model for the tracked object
    failure=readInitialmodel3dPoints(_model3dPointsMat,trackedObjectShapeTemplate);
    if(failure)
    {
        cout<<"I had troubles reading the model 3D points."<<endl;
        quit=true;
    }

    //create _visualization3dPointsMat and fill it with the average between outer and inner 3D points.
    _visualization3dPointsMat=cv::Mat( 3, 2*nPixels, CV_64FC1 );
    //only the first half of this matrix is used. the second part can be full of rubbish (not zeros, I guess).
    _visualization3dPointsMat.setTo(cv::Scalar(1));
    for(row=0;row<3;row++)
    {
        for(column=0;column<nPixels;column++)
        {
            
            ((double*)(_visualization3dPointsMat.data + _visualization3dPointsMat.step*row))[column]=(((double*)(_model3dPointsMat.data + _model3dPointsMat.step*row))[column]+((double*)(_model3dPointsMat.data + _model3dPointsMat.step*row))[column+nPixels])/2;
        }
    }


    //Read the motion model matrix for the tracked object
    _A=cvCreateMat(7,7,CV_64FC1);    //allocate space for the _A matrix. 64bit double, 1 channel
    failure=readMotionModelMatrix(_A, motionModelMatrix);
    if(failure)
    {
        cout<<"I had troubles reading the motion model matrix."<<endl;
        quit=true;
    }

    //allocate memory for the particles;
    _particles=cv::Mat(7,_nParticles,CV_64FC1);
    //fill the memory with zeros, so that valgrind won't complain.
    _particles.setTo(cv::Scalar(0));

    //define ways of accessing the particles:
    _particles1=_particles(cv::Range(0,1), cv::Range::all());
    _particles2=_particles(cv::Range(1,2), cv::Range::all());
    _particles3=_particles(cv::Range(2,3), cv::Range::all());
    _particles4=_particles(cv::Range(3,4), cv::Range::all());
    _particles5=_particles(cv::Range(4,5), cv::Range::all());
    _particles6=_particles(cv::Range(5,6), cv::Range::all());
    _particles7=_particles(cv::Range(6,7), cv::Range::all());
    _particles1to6=_particles(cv::Range(0,6), cv::Range::all());
    //allocate memory for the "new" particles;
    _newParticles=cv::Mat(7,_nParticles,CV_64FC1);
    _newParticles1to6=_newParticles(cv::Range::all(), cv::Range::all());
    //allocate memory for "noise"
    _noise=cv::Mat(6,_nParticles,CV_64FC1);
    _noise.setTo(cv::Scalar(0));
    _noise1 = _noise(cv::Range(0,3), cv::Range::all());
    _noise2 = _noise(cv::Range(3,6), cv::Range::all());
    _noise1.setTo(cv::Scalar(0));
    _noise2.setTo(cv::Scalar(0));

    //resampling-related stuff.
    _nChildren = cv::Mat(1,_nParticles,CV_64FC1);
    _label     = cv::Mat(1,_nParticles,CV_64FC1);
    _ramp      = cv::Mat(1,_nParticles,CV_64FC1);
    _u         = cv::Mat(1,_nParticles,CV_64FC1);

    _cumWeight =cv::Mat(1,_nParticles+1,CV_64FC1);

    int count;
    for(count=0;count<_nParticles;count++)
    {
        ((double*)(_ramp.data))[count]=(double)count+1.0F;
    }




    if(_initializationMethod=="3dEstimate") //TODO make this the only way initialization can be done
    {
        //cout<<"Initialization method = 3dEstimate."<<endl;
        //*************************************************************************
        //generate a set of random particles near the estimated initial 3D position
        //*************************************************************************

        double mean,velocityStDev;
        velocityStDev=0; //warning ??? !!! I'm setting parameters for the dynamic model here.

        //initialize X
        mean=(double)_initialX;
        cv::randn(_particles1, cv::Scalar(mean), cv::Scalar(_accelStDev));
        //initialize Y
        mean=(double)_initialY;
        cv::randn(_particles2, cv::Scalar(mean), cv::Scalar(_accelStDev));
        //initialize Z
        mean=(double)_initialZ;
        cv::randn(_particles3, cv::Scalar(mean), cv::Scalar(_accelStDev));

        //initialize VX
        mean=0;
        cv::randn(_particles4, cv::Scalar(mean), cv::Scalar(velocityStDev));

        //initialize VY
        cv::randn(_particles5, cv::Scalar(mean), cv::Scalar(velocityStDev));

        //initialize VZ
        cv::randn(_particles6, cv::Scalar(mean), cv::Scalar(velocityStDev));

    }

    downsampler=0; //this thing is used to send less data to the plotter TODO remove


    //Matrices-related stuff.
    //connect headers to data, allocate space...
    _rzMat = cv::Mat(3, 3, CV_64FC1);
    _ryMat = cv::Mat(3, 3, CV_64FC1);
    _uv = cv::Mat(2,2*nPixels, CV_64FC1);

    _tempMat1=_tempMat(cv::Range(0,1), cv::Range::all());
    _tempMat2=_tempMat(cv::Range(1,2), cv::Range::all());
    _tempMat3=_tempMat(cv::Range(2,3), cv::Range::all());

    _p2Mat1=_points2Mat(cv::Range(0,1),cv::Range::all());
    _p2Mat3=_points2Mat(cv::Range(2,3),cv::Range::all());

    _drawingMat=cv::Mat(3, 2*nPixels, CV_64FC1);
    _projectionMat=cv::Mat(2, 3, CV_64FC1);

    _xyzMat1 = cvCreateMatHeader(1,2*nPixels,CV_64FC1);
    _xyzMat2 = cvCreateMatHeader(1,2*nPixels,CV_64FC1);
    _xyzMat3 = cvCreateMatHeader(1,2*nPixels,CV_64FC1);



    _framesNotTracking=0;
    _frameCounter=1;
    _attentionOutput=0;
    _firstFrame=true;

    // Set the output ports
    _outputVideoPort = it_.advertise(_outputVideoPortName.c_str(), 1);
    _outputDataPort  = n_.advertise<pf3d_tracker::Estimates>(_outputDataPortName, 1);
    
    // Set the input video port, with the associated callback method
    _inputVideoPort = it_.subscribe(_inputVideoPortName.c_str(), 1, &PF3DTracker::processImageCallback, this);
    if(quit==true)
    {
        printf("There were problems initializing the object: the execution was interrupted.\n");
        fflush(stdout);
        return false; //there were problems initializing the objet: stop the execution.
    }
    else
    {
        //Write the header for the output data
        cout<<"  frame#";
        cout<<"   meanX";
        cout<<"   meanY";
        cout<<"   meanZ";
        cout<<"   Likelihood";
        cout<<"   Seing";
        cout<<"   meanU";
        cout<<"   meanV";
        cout<<"   fps"<<endl;

        _doneInitializing=true;
        return true;  //the object was set up successfully.
    }

    

}






//**************************
//* PROCESS IMAGE CALLBACK *
//**************************
void PF3DTracker::processImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    tic();
    int count;
    unsigned int seed;
    double likelihood, mean, maxX, maxY, maxZ;
    double weightedMeanX, weightedMeanY, weightedMeanZ;
    double meanU;
    double meanV;
    double wholeCycle;
    string outputFileName;
    stringstream out;
    cv::Mat rawImageBGR;

    seed=rand();
    
    if(_staticImageTest)
    {
        rawImageBGR = cv::imread( "testImage.png", 1) ;
        if( rawImageBGR.data == 0 ) //load the image from file.
        {
            std::cout << "Tried to open testImage.png"<<std::endl;
            cout<<"I wasn't able to open the test image file!\n";
            fflush(stdout);
            return; //if I can't do it, I just quit the program.
        }

        //_rawImage = cvCreateImage(cvSize(rawImageBGR.cols, rawImageBGR.rows), rawImageBGR->depth, rawImageBGR->nChannels);
        //cvCvtColor(rawImageBGR,_rawImage,CV_BGR2RGB);
        _rawImage=rawImageBGR;
    }
    else
    {
        //Read the image from the buffer
        try
        {
            std::string aux = "bgr8";

            _rawImage=cv_bridge::toCvShare(msg_ptr, aux)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //_rawImage = bridge_.imgMsgToCv(msg_ptr,aux); //This is an RGB image
    }



    //         //TEST write the bytes of this image to a file, so that we see what the encoding is.
    //         int z1,z2;
    //         ofstream myfile;
    //         myfile.open ("channel0.txt");
    //         for(z1=0;z1<_rawImage.rows;z1++)
    //         {
    //           for(z2=0;z2<_rawImage.cols;z2++)
    //           {
    //              myfile << (int)(((uchar*)(_rawImage.data + _rawImage.step*z1))[z2*3+0])<< ", ";
    //           }
    //              myfile << endl;
    //         }
    //         myfile.close();
    //         myfile.open ("channel1.txt");
    //         for(z1=0;z1<_rawImage.rows;z1++)
    //         {
    //           for(z2=0;z2<_rawImage.cols;z2++)
    //           {
    //              myfile << (int)(((uchar*)(_rawImage.data + _rawImage.step*z1))[z2*3+1])<< ", ";
    //           }
    //              myfile << endl;
    //         }
    //         myfile.close();
    //         myfile.open ("channel2.txt");
    //         for(z1=0;z1<_rawImage.rows;z1++)
    //         {
    //           for(z2=0;z2<_rawImage.cols;z2++)
    //           {
    //              myfile << (int)(((uchar*)(_rawImage.data + _rawImage.step*z1))[z2*3+2])<< ", ";
    //           }
    //              myfile << endl;
    //         }
    //         myfile.close();




    //*****************************************
    //calculate the likelihood of each particle
    //*****************************************
    double sumLikelihood=0.0;
    double maxLikelihood=0.0;
    int   maxIndex=-1;
    for(count=0;count< _nParticles;count++)
    {

        evaluateHypothesisPerspectiveFromRgbImage(_model3dPointsMat,(double)_particles.at<double>(0,count),(double)_particles.at<double>(1,count),(double)_particles.at<double>(2,count),_modelHistogramMat,_rawImage,_perspectiveFx,_perspectiveFy, _perspectiveCx,_perspectiveCy,_insideOutsideDifferenceWeight,likelihood);

        _particles.at<double>(6,count)=likelihood;
        sumLikelihood+=likelihood;
        if(likelihood>maxLikelihood)
        {
            maxLikelihood=likelihood;
            maxIndex=count;
        }
    }
    
    
    if(maxIndex!=-1)
    {
        maxX=(double)_particles.at<double>(0,maxIndex);
        maxY=(double)_particles.at<double>(1,maxIndex);
        maxZ=(double)_particles.at<double>(2,maxIndex);
    }
    else
    {
        maxX=1;
        maxY=1;
        maxZ=1000;
    }
    
    if(_staticImageTest)
    {
        cout<<"maxLikelihood = "<<maxLikelihood<<endl;
        cout<<"likelihood for comparison = "<<maxLikelihood/exp((double)20.0)<<endl;
    }
    if(maxLikelihood/exp((double)20.0)>_likelihoodThreshold) //normalizing likelihood
    {
        _seeingObject=1;
        _framesNotTracking=0;
        _attentionOutput=_attentionOutputMax;
    }
    else
    {
        _attentionOutput=_attentionOutput*_attentionOutputDecrease;
        _seeingObject=0;
        _framesNotTracking+=1;
    }
    
    
    //If the likelihood has been under the threshold for 5 frames, reinitialize the tracker.
    //This just works for the sphere.
    if(_framesNotTracking==5 || sumLikelihood==0.0)
    {
        cout<<"**********************************************************************Reset\n";
        double mean,velocityStDev;
        velocityStDev=0; //warning ??? !!! I'm setting parameters for the dynamic model here.

        mean=(double)_initialX;
        cv::randn(_particles1, cv::Scalar(mean), cv::Scalar(_accelStDev));

        //initialize Y
        mean=(double)_initialY;
        cv::randn(_particles2, cv::Scalar(mean), cv::Scalar(_accelStDev));

        //initialize Z
        mean=(double)_initialZ;
        cv::randn(_particles3, cv::Scalar(mean), cv::Scalar(_accelStDev));

        //initialize VX
        mean=0;
        cv::randn(_particles4, cv::Scalar(mean), cv::Scalar(velocityStDev));
        //initialize VY
        cv::randn(_particles5, cv::Scalar(mean), cv::Scalar(velocityStDev));
        //initialize VZ
        cv::randn(_particles6, cv::Scalar(mean), cv::Scalar(velocityStDev));

        _framesNotTracking=0;

        weightedMeanX=weightedMeanY=weightedMeanZ=0.0;    // UGO: they should be zeroed before accumulation
        for(count=0;count<_nParticles;count++)
        {

            weightedMeanX+=(double)_particles.at<double>(0,count);
            weightedMeanY+=(double)_particles.at<double>(1,count);
            weightedMeanZ+=(double)_particles.at<double>(2,count);
        }
        weightedMeanX/=_nParticles;
        weightedMeanY/=_nParticles;
        weightedMeanZ/=_nParticles;
        //this mean is not weighted as there is no weight to use: the particles have just been generated.


        //*****************************************
        //WRITE ESTIMATES TO THE SCREEN, FIRST PART
        //*****************************************
        //these are not really estimates, but...
        cout<<setw(8)<<_frameCounter;
        cout<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanX/1000; //millimeters to meters
        cout<<"  "<<setw(8)<<weightedMeanY/1000; //millimeters to meters
        cout<<"  "<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanZ/1000; //millimeters to meters
        cout<<"  "<<setiosflags(ios::fixed)<<setprecision(5)<<setw(8)<<maxLikelihood/exp((double)20.0); //normalizing likelihood
        cout<<"  "<<setw(5)<<_seeingObject;
    }
    else
    {
        //*********************************************
        //Compute the mean and normalize the likelihood
        //*********************************************
        weightedMeanX=0.0;
        weightedMeanY=0.0;
        weightedMeanZ=0.0;
        for(count=0;count<_nParticles;count++)
        {
            _particles.at<double>(6,count)=_particles.at<double>(6,count)/sumLikelihood;
            weightedMeanX+=(double)_particles.at<double>(0,count)*_particles.at<double>(6,count);
            weightedMeanY+=(double)_particles.at<double>(1,count)*_particles.at<double>(6,count);
            weightedMeanZ+=(double)_particles.at<double>(2,count)*_particles.at<double>(6,count);
        }



        //*****************************
        //WRITE ESTIMATES TO THE SCREEN
        //*****************************
        cout<<setw(8)<<_frameCounter;
        cout<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanX/1000; //millimeters to meters
        cout<<"  "<<setw(8)<<weightedMeanY/1000; //millimeters to meters
        cout<<"  "<<setiosflags(ios::fixed)<<setprecision(3)<<setw(8)<<weightedMeanZ/1000; //millimeters to meters
        cout<<"  "<<setiosflags(ios::fixed)<<setprecision(5)<<setw(8)<<maxLikelihood/exp((double)20.0); //normalizing likelihood
        cout<<"  "<<setw(5)<<_seeingObject;


        //------------------------------------------------------------martim
        /*Bottle *particleInput=_inputParticlePort.read(false);
         if(particleInput==NULL) _numParticlesReceived=0;
         else _numParticlesReceived=(particleInput->get(0)).asInt();
         if(_numParticlesReceived > _nParticles){
           _numParticlesReceived=0;
           cout<<"PROBLEM: Input particles are more than nParticles.\n";
         }*/
        //------------------------------------------------------------end martim

        //**********************
        //RESAMPLE THE PARTICLES
        //**********************
        //cout<<"T1\n";
        //fflush(stdout);
        int minimum_likelihood=10; //do not resample if maximum likelihood is lower than this.
        //this is intended to prevent that the particles collapse on the origin when you start the tracker.
        if(maxLikelihood>minimum_likelihood)
        {
            //fflush(stdout);

            //TODO non funziona ancora, credo: nelle particelle resamplate ci sono dei not-a-number.
            systematic_resampling(_particles1to6,_particles7,_newParticles,_cumWeight);

        }
        else //I can't apply a resampling with all weights equal to 0!
        {
            //fflush(stdout);

            //TODO:CHECK that copying the whole thing creates no problems.
            //I think I used to copy only 6 lines to make it faster.
            //Copy(&_particles[0][0], &_newParticles[0][0], 6*_nParticles);
            _particles.copyTo(_newParticles);
            //cvCopy(_particles,_newParticles);
        }

        //cout<<"after resampling\n";
        /*            cout<<"Accessing the first column of _NewParticles after the resampling: "<<((double*)(_newParticles.data +  + _newParticles.step*0))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*1))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*2))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*3))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*4))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*5))[0]<<" ";
            cout<<((double*)(_newParticles.data + _newParticles.step*6))[0]<<endl;*/

        //printMat(_newParticles);

        //printMat(_A);









        //the "good" particles now are in _newParticles
        //******************************************
        //APPLY THE MOTION MODEL: 1.APPLY THE MATRIX
        //******************************************
        cv::multiply(_A,_newParticles,_particles);

        //the "good" particles now are in _particles
        //********************************************************
        //APPLY THE MOTION MODEL: 2.ADD THE EFFECT OF ACCELERATION
        //********************************************************
        mean = 0;
        //cout<<"Noise generation parameters: mean= "<<mean<<", accelStDev= "<<_accelStDev<<endl;
        //cout<<"_noise1 before generation: "<<((double*)(_noise1.data + _noise.step*0))[0]<<endl;
        cv::randn(_noise1, cv::Scalar(mean), cv::Scalar(_accelStDev));

        //cout<<"_noise1 after generation: "<<((double*)(_noise1.data + _noise.step*0))[0]<<endl;
        _noise1.copyTo(_noise2);
        //cvConvertScale( _noise1, _noise1, 0.5, 0 );//influence on the position is half that on speed.
        _noise1*=0.5;
        //cout<<"_noise1 after rescaling: "<<((double*)(_noise1.data + _noise.step*0))[0]<<endl;

        //cout<<"_noise2 after generation: "<<((double*)(_noise2.data + _noise.step*0))[0]<<endl;

        //cout<<"First element of _particles before addition of noise: "<<((double*)(_particles.data + _particles.step*0))[0]<<endl;
        cv::add(_particles1to6,_noise,_particles1to6);//sum the influence of the noise to the previous status
        //cout<<"First element of _particles after addition of noise: "<<((double*)(_particles.data + _particles.step*0))[0]<<endl;




        //------------------------------------------------------------martim
        // get particles from input
        /*if(_numParticlesReceived > 0){
            int topdownParticles = _nParticles - _numParticlesReceived;
            for(count=0 ; count<_numParticlesReceived ; count++){
                cvmSet(_particles,0,topdownParticles+count, (particleInput->get(1+count*3+0)).asDouble());
                cvmSet(_particles,1,topdownParticles+count, (particleInput->get(1+count*3+1)).asDouble());
                cvmSet(_particles,2,topdownParticles+count, (particleInput->get(1+count*3+2)).asDouble());
                cvmSet(_particles,3,topdownParticles+count, 0);
                cvmSet(_particles,4,topdownParticles+count, 0);
                cvmSet(_particles,5,topdownParticles+count, 0);
                cvmSet(_particles,6,topdownParticles+count, 0.8); //??
            }
            //num_bottomup_objects=(particleInput->get(1+count*3)).asInt();
        }*/
        //------------------------------------------------------------end martim
    }
    

    //************************************
    //DRAW THE SAMPLED POINTS ON THE IMAGE
    //************************************
    if(_seeingObject)
    {
        drawContourPerspective(_visualization3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _rawImage, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 0, 255, 0, meanU, meanV);
        drawSampledLinesPerspective(_model3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _rawImage,_perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 255, 255, 255, meanU, meanV);
    }
    else
    {
        drawContourPerspective(_visualization3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _rawImage, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 0,255, 255, meanU, meanV);
        //Red+Green=yellow
        drawSampledLinesPerspective(_model3dPointsMat, weightedMeanX,weightedMeanY,weightedMeanZ, _rawImage,_perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, 255, 255, 255, meanU, meanV);
    }


    wholeCycle=1.0/tac();
    cout<<setw(8)<<(int)meanU;
    cout<<setw(5)<<(int)meanV;
    if(_firstFrame==false)
    {
        cout<<setw(5)<<setw(8)<<setiosflags(ios::fixed)<<setprecision(3)<<wholeCycle<<endl;
    }
    else
    {
        cout<<"   -----"<<endl;
        _firstFrame=false;
    }
    
    /////////////////
    // DATA OUTPUT //
    /////////////////

    pf3d_tracker::Estimates outMsg;
    outMsg.mean.point.x=weightedMeanX/1000;
    outMsg.mean.point.y=weightedMeanY/1000;
    outMsg.mean.point.z=weightedMeanZ/1000;

    outMsg.likelihood=maxLikelihood/exp((double)20.0);
    outMsg.meanU=meanU;
    outMsg.meanV=meanV;
    outMsg.seeingBall=_seeingObject;
    _outputDataPort.publish(outMsg);



    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _rawImage).toImageMsg();

    _outputVideoPort.publish(img_msg);

    if(_staticImageTest)
    {
        cv::imwrite("testImageOutput.png", _rawImage);
        ros::shutdown(); //exit the program after just one cycle
    }
    //cvReleaseImage(&_rawImage);
    _frameCounter++;
    
}





//destructor
PF3DTracker::~PF3DTracker()
{
    cout<<"Stopping pf3dTracker"<<endl;
}




//member that closes the object.
bool PF3DTracker::close()
{

    //cvReleaseMat(&_A);
    //cvReleaseMat(&_particles);
    //cvReleaseMat(&_newParticles);

    return true;
}





void PF3DTracker::drawContourPerspective(cv::Mat & model3dPointsMat,double x, double y, double z, cv::Mat & image, double _perspectiveFx,double  _perspectiveFy ,double _perspectiveCx,double  _perspectiveCy ,int R, int G, int B, double &meanU, double &meanV)
{

    //IplImage image =cv_ptr->image;

    bool failure;
    //cv::Mat & uv=cvCreateMat(2,2*nPixels,CV_64FC1);

    //create a copy of the 3D original points.
    model3dPointsMat.copyTo(_drawingMat);


    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
    failure=place3dPointsPerspective(_drawingMat,x,y,z);
    //cout<<"rototraslated points:\n";
    //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);





    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
    failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv)!=0;
    if(failure)
    {
        cout<<"I had troubles projecting the points.\n";
    }
    //used to be:
    //         //(Ipp32f *xyz, int xyzColumns, int xyzRows, int xyzStride1, int xyzStride2, double fx, double fy, Ipp32f u0, Ipp32f v0, Ipp32f *uv, int uvStride1, int uvStride2
    //         failure=perspective_projection(&model3dPointsDuplicate[0][0],2*nPixels,3,sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f),_perspectiveFx,  _perspectiveFy , _perspectiveCx,  _perspectiveCy,&uv[0][0],sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f));
    //
    //         if(failure)
    //         {
    //             cout<<"I had troubles projecting the points.\n";
    //         }



    //****
    //Draw
    //****
    int conta,cippa,lippa,uPosition,vPosition;
    meanU=0;
    meanV=0;
    for(conta=0;conta<nPixels;conta++)
    {
        meanV=meanV+((double*)(_uv.data + _uv.step*1))[conta];
        meanU=meanU+((double*)(_uv.data + _uv.step*0))[conta];

        for(lippa=-2;lippa<3;lippa++)
            for(cippa=-2;cippa<3;cippa++)
            {
                vPosition= (int)(((double*)(_uv.data + _uv.step*1))[conta])+lippa-1;
                uPosition= (int)(((double*)(_uv.data + _uv.step*0))[conta])+cippa-1;

                if((uPosition<_rawImage.cols)&&(uPosition>=0)&&(vPosition<_rawImage.rows)&&(vPosition>=0))
                {
                    // NOT SURE IF WIDTHSTEP OR WIDTH
                    (((uchar*)(_rawImage.data + _rawImage.step*vPosition))[uPosition*3+0])=R;
                    (((uchar*)(_rawImage.data + _rawImage.step*vPosition))[uPosition*3+1])=G;
                    (((uchar*)(_rawImage.data + _rawImage.step*vPosition))[uPosition*3+2])=B;
                }

            }
    }

    meanU=floor(meanU/nPixels);
    meanV=floor(meanV/nPixels);
    if((meanU<_rawImage.cols)&&(meanU>=0)&&(meanV<_rawImage.rows)&&(meanV>=0))
    {
        (((uchar*)(_rawImage.data + _rawImage.step*(int)meanV))[(int)meanU*3+0])=R;
        (((uchar*)(_rawImage.data + _rawImage.step*(int)meanV))[(int)meanU*3+1])=G;
        (((uchar*)(_rawImage.data + _rawImage.step*(int)meanV))[(int)meanU*3+2])=B;
    }

}





bool PF3DTracker::computeTemplateHistogram(string imageFileName,string dataFileName)
{
    int u,v,a,b,c;
    double usedPoints=0;
    //double histogram[YBins][UBins][VBins];
    //double* histogram;
    //histogram = new double[YBins*UBins*VBins]
    int dimensions;
    dimensions=3;
    int sizes[3]={YBins,UBins,VBins};
    //create histogram and allocate memory for it.
    CvMatND* histogram=cvCreateMatND(dimensions, sizes, CV_64FC1);
    if(histogram==0)
    {
        cout<<"computeTemplateHistogram: I wasn\'t able to allocate memory for histogram.\n";
        fflush(stdout);
        return true; //if I can't do it, I just quit the program.
    }
    //set content of the matrix to zero.
    cvSetZero(histogram);//used to be: ipps Zero_32f(&histogram[0][0][0], YBins*UBins*VBins);
    cv::Mat rawImage;
    cv::Mat rawImageRGB;
    cv::Mat transformedImage;
    
    //load the image
    rawImage = cv::imread( imageFileName, CV_LOAD_IMAGE_COLOR); //load the image from file.

    if(! rawImage.data )
    {
        std::cout << "tried to open: " <<  imageFileName.c_str() <<std::endl;
        cout<<"I wasn't able to open the image file!\n";
        fflush(stdout);
        return true; //if I can't do it, I just quit the program.
    }


    rawImageRGB =cv::Mat(rawImage.rows, rawImage.cols,rawImage.type());
    cv::cvtColor(rawImage,rawImageRGB,CV_BGR2RGB);
    
    //allocate space for the transformed image
    transformedImage = cv::Mat(rawImage.rows, rawImage.cols,rawImage.type());//cvCreateImage(cvSize(rawImage.cols,rawImage.rows),IPL_DEPTH_8U,3);

    //transform the image in the YUV format
    rgbToYuvBinImageLut(rawImage,transformedImage,_lut);
    
    //count the frequencies of colour bins, build the histogram.
    for(v=0;v<rawImage.rows;v++)
        for(u=0;u<rawImage.cols;u++)
        {
            //discard white pixels [255,255,255].
            if(!(
                        (((uchar*)(rawImage.data + rawImage.step*v))[u*3+0])==255 && (((uchar*)(rawImage.data + rawImage.step*v))[u*3+1])==255 && (((uchar*)(rawImage.data + rawImage.step*v))[u*3+2])==255)

                    )
            {


                a=(((uchar*)(transformedImage.data + transformedImage.step*v))[u*3+0]);//Y bin
                b=(((uchar*)(transformedImage.data + transformedImage.step*v))[u*3+1]);//U bin
                c=(((uchar*)(transformedImage.data + transformedImage.step*v))[u*3+2]);//V bin

                //TEST printf("histogram->size[0].step,%d\n",histogram->dim[0].step);  256
                //TEST printf("histogram->size[1].step,%d\n",histogram->dim[1].step);   32
                //TEST printf("histogram->size[2].step,%d\n",histogram->dim[2].step);    4
                *((double*)(histogram->data + a*histogram->dim[0].step + b*histogram->dim[1].step + c*histogram->dim[2].step)) +=1;

                //initial pointer + Y*UBINS*VBINS*histogram.step + U*VBINS*histogram.step + V*histogram.step. RIGHT?
                //histogram[(yuvBinsImage[u][v][0])*UBins*VBins + (yuvBinsImage[u][v][1])*VBins +  (yuvBinsImage[u][v][2]) ]+=1; //increment the correct bin counter.
                usedPoints+=1;
            }
            
        }

    //normalize
    if(usedPoints>0)
    {
        //histogram=histogram/usedPoints
        cvConvertScale( histogram, histogram, 1/usedPoints, 0 );
        //used to be: ipps DivC_32f_I(usedPoints, &histogram[0][0][0], YBins*UBins*VBins);
    }

    //write the computed histogram to a file.
    ofstream fout(dataFileName.c_str());//open file
    if(!fout)                           //confirm file opened
    {
        cout << "computeTemplateHistogram: unable to open the csv file to store the histogram.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(a=0;a<YBins;a++)
        {
            for(b=0;b<UBins;b++)
            {
                for(c=0;c<VBins;c++)
                {
                    fout<<*((double*)(histogram.data + a*histogram->dim[0].step + b*histogram->dim[1].step + c*histogram->dim[2].step))<<endl;
                    //used to be: fout<<(double)(histogram[a*UBins*VBins + b*VBins + c])<<endl;
                }
            }
        }
        fout.close();
    }

    //clean memory up
    cvReleaseMatND(&histogram);

    return false;
}

bool PF3DTracker::readModelHistogram(CvMatND* histogram,const char fileName[])
{
    int c1,c2,c3;
    char line[15];

    ifstream fin(fileName);//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the csv histogram file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<YBins;c1++)
            for(c2=0;c2<UBins;c2++)
                for(c3=0;c3<VBins;c3++)
                {
                    fin.getline(line, 14);
                    *((double*)(histogram.data + c1*histogram->dim[0].step + c2*histogram->dim[1].step + c3*histogram->dim[2].step))=(double)atof(line);
                    //TEST cout << c1 <<" "<<c2<<" "<<c3 << endl;
                    //TEST fflush(stdout);
                }
        return false;
    }
}



bool PF3DTracker::readInitialmodel3dPoints(cv::Mat & points, string fileName)
{
    int c1,c2;
    char line[15];

    ifstream fin(fileName.c_str());//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the the 3D model file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<3;c1++)
            for(c2=0;c2<2*nPixels;c2++)
            {
                fin.getline(line, 14);
                ((double*)(points.data + points.step*c1))[c2]=(double)atof(line);
            }
        return false;
    }
}



bool PF3DTracker::readMotionModelMatrix(cv::Mat & points, string fileName)
{
    int c1,c2;
    char line[15];

    ifstream fin(fileName.c_str());//open file
    if(!fin)                           //confirm file opened
    {
        cout << "unable to open the motion model file.\n" << endl;
        fflush(stdout);
        return true;
    }
    else
    {
        for(c1=0;c1<7;c1++)
            for(c2=0;c2<7;c2++)
            {
                fin.getline(line, 14);
                cvmSet(points,c1,c2,atof(line));

            }
        return false;
    }
}






bool PF3DTracker::evaluateHypothesisPerspectiveFromRgbImage(cv::Mat & model3dPointsMat,double x, double y, double z, CvMatND* modelHistogramMat, cv::Mat & image,  double fx, double fy, double u0, double v0, double inside_outside, double &likelihood)
{

    bool failure;
    double usedOuterPoints, usedInnerPoints;

    //create a copy of the 3D original points.
    model3dPointsMat.copyTo(_drawingMat);

    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
    failure=place3dPointsPerspective(_drawingMat,x,y,z);

    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
    failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv)!=0;
    if(failure)
    {
        cout<<"I had troubles projecting the points.\n";
    }
    


    
    //IT SEEMS TO BE THIS GUY COMPUTING THE WRONG VALUES
    computeHistogramFromRgbImage(_uv, image,  _innerHistogramMat, usedInnerPoints, _outerHistogramMat, usedOuterPoints);
    
    
    
    
    //if((usedInnerPoints<nPixels)||(usedOuterPoints<nPixels))
    //    likelihood=0;
    //else
    failure=calculateLikelihood(_modelHistogramMat, _innerHistogramMat, _outerHistogramMat, inside_outside,likelihood);
    
    likelihood=exp(20*likelihood); //no need to divide: I'm normalizing later.
    if(_staticImageTest)
    {
        cout<<"exp likelihood = "<<likelihood<<endl;
    }
    //make hypotheses with pixels outside the image less likely.
    likelihood=likelihood*((double)usedInnerPoints/nPixels)*((double)usedInnerPoints/nPixels)*((double)usedOuterPoints/nPixels)*((double)usedOuterPoints/nPixels);
    if(_staticImageTest)
    {
        cout<<"likelihood compensating for non-visible points = "<<likelihood<<endl;
    }
    return false;
}



bool PF3DTracker::systematic_resampling(cv::Mat & oldParticlesState, cv::Mat & oldParticlesWeights, cv::Mat & newParticlesState, cv::Mat & cumWeight)
{
    //function [newParticlesState] = systematic_resampling(oldParticlesWeight, oldParticlesState)

    double u; //random number [0,1)
    double sum;
    int c1;
    int rIndex;  //index of the randomized array
    int cIndex;  //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    int npIndex; //%new particle index, tells me how many particles have been created so far.
    int numParticlesToGenerate = _nParticles - _numParticlesReceived; //martim

    //%N is the number of particles.
    //[lines, N] = size(oldParticlesWeight);
    //in CPP, _nParticles is the number of particles.


    //%NORMALIZE THE WEIGHTS, so that sum(oldParticles)=1.
    //oldParticlesWeight = oldParticlesWeight / sum(oldParticlesWeight);
    sum=0;
    for(c1=0;c1<_nParticles;c1++)
    {
        sum+=((double*)(oldParticlesWeights.data + oldParticlesWeights.step*0))[c1];
    }
    for(c1=0;c1<_nParticles;c1++)
    {
        ((double*)(oldParticlesWeights.data + oldParticlesWeights.step*0))[c1] = (((double*)(oldParticlesWeights.data + oldParticlesWeights.step*0))[c1])/(double)sum;
    }


    //%GENERATE N RANDOM VALUES
    //u = rand(1)/N; %random value [0,1/N)
    u=1/(double)numParticlesToGenerate*((double)rand()/(double)RAND_MAX); //martim

    //%the randomized values are going to be u, u+1/N, u+2/N, etc.
    //%instread of accessing this vector, the elements are computed on the fly:
    //%randomVector(a)= (a-1)/N+u.

    //%COMPUTE THE ARRAY OF CUMULATIVE WEIGHTS
    //cumWeight=zeros(1,N+1);
    //cumWeight[0]=0;
    ((double*)(cumWeight.data))[0]=0;
    for(c1=0;c1<_nParticles;c1++)
    {
        //cumWeight[c1+1]=cumWeight[c1]+oldParticlesWeight[c1];

        ((double*)(cumWeight.data))[c1+1]=((double*)(cumWeight.data))[c1]+((double*)(oldParticlesWeights.data + oldParticlesWeights.step*0))[c1];
        //cout<<"cumulative at position "<<c1+1<<" = "<<((double*)(cumWeight.data))[c1+1]<<endl;

    }
    //CHECK IF THERE IS SOME ROUNDING ERROR IN THE END OF THE ARRAY.
    //if(cumWeight[_nParticles]!=1)
    if(((double*)(cumWeight.data))[_nParticles]!=1)
    {
        //fprintf('rounding error?\n');
        //printf("cumWeight[_nParticles]==%15.10e\n",((double*)(cumWeight.data))[_nParticles]);
        ((double*)(cumWeight.data))[_nParticles]=1;
        if( ((double*)(cumWeight.data))[_nParticles]!=1)
        {
            //printf("still different\n");
        }
        else
        {
            //printf("now it-s ok\n");
        }
    }

    //cout<<"cumulative at position "<<_nParticles-1<<" = "<<((double*)(cumWeight.data))[_nParticles-1]<<endl;
    //cout<<"cumulative at position "<<_nParticles<<" = "<<((double*)(cumWeight.data))[_nParticles]<<endl;

    //%PERFORM THE ACTUAL RESAMPLING
    rIndex=0; //index of the randomized array
    cIndex=1; //index of the cumulative weight array. cIndex -1 indicates which particle we think of resampling.
    npIndex=0; //new particle index, tells me how many particles have been created so far.

    while(npIndex < numParticlesToGenerate) //martim
    {
        //siamo sicuri che deve essere >=? ??? !!! WARNING
        if(((double*)(cumWeight.data))[cIndex]>=(double)rIndex/(double)numParticlesToGenerate+u) //martim
        {
            //%particle cIndex-1 should be copied.
            //printf("replicating particle %d\n",cIndex-1);
            //newParticlesState(npIndex)=oldParticlesState(cIndex-1);
            ((double*)(newParticlesState.data + newParticlesState.step*0))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*0))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*1))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*1))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*2))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*2))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*3))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*3))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*4))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*4))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*5))[npIndex]=((double*)(oldParticlesState.data + oldParticlesState.step*5))[cIndex-1];
            ((double*)(newParticlesState.data + newParticlesState.step*6))[npIndex]=0; //initializing weight
            rIndex=rIndex+1;
            npIndex=npIndex+1;
        }
        else
        {
            //printf("not replicating particle %d\n",cIndex-1);
            cIndex=cIndex+1;
        }
    }
    
    return false;
}







bool PF3DTracker::place3dPointsPerspective(cv::Mat & points, double x, double y, double z)
{

    
    //*********************
    // 0. Prepare some data
    //*********************
    double floorDistance=sqrt(x*x+y*y);      //horizontal distance from the optical center to the ball
    double distance=sqrt(x*x+y*y+z*z);       //distance from the optical center to the ball
    
    double cosAlpha=floorDistance/distance;  //cosine of an angle needed for a rotation
    double sinAlpha=-z/distance;             //sine of an angle needed for a rotation
    double cosBeta=x/floorDistance;          //cosine of an angle needed for a rotation
    double sinBeta=y/floorDistance;          //sine of an angle needed for a rotation

    
    //Rotation matrix Rz: [3 x 3]
    ((double*)(_rzMat.data + _rzMat.step*0))[0]=  cosBeta;
    ((double*)(_rzMat.data + _rzMat.step*0))[1]= -sinBeta;
    ((double*)(_rzMat.data + _rzMat.step*0))[2]=        0;
    ((double*)(_rzMat.data + _rzMat.step*1))[0]=  sinBeta;
    ((double*)(_rzMat.data + _rzMat.step*1))[1]=  cosBeta;
    ((double*)(_rzMat.data + _rzMat.step*1))[2]=        0;
    ((double*)(_rzMat.data + _rzMat.step*2))[0]=        0;
    ((double*)(_rzMat.data + _rzMat.step*2))[1]=        0;
    ((double*)(_rzMat.data + _rzMat.step*2))[2]=        1;


    //Rotation matrix Ry: [3 x 3]
    ((double*)(_ryMat.data + _ryMat.step*0))[0]=  cosAlpha;
    ((double*)(_ryMat.data + _ryMat.step*0))[1]=         0;
    ((double*)(_ryMat.data + _ryMat.step*0))[2]=  sinAlpha;
    ((double*)(_ryMat.data + _ryMat.step*1))[0]=         0;
    ((double*)(_ryMat.data + _ryMat.step*1))[1]=         1;
    ((double*)(_ryMat.data + _ryMat.step*1))[2]=         0;
    ((double*)(_ryMat.data + _ryMat.step*2))[0]= -sinAlpha;
    ((double*)(_ryMat.data + _ryMat.step*2))[1]=         0;
    ((double*)(_ryMat.data + _ryMat.step*2))[2]=  cosAlpha;
    
    


    //***********************************
    // 1. Rotate points around the Y axis
    //***********************************
    //Multiply Ry by points
    //_points2Mat=_ryMat*points     [3 x 2*nPixels]
    cv::multiply(_ryMat,points,_points2Mat);


    //used to be:
    /*
        returnValue=ippmMul_mm_32f(ry,      rzStride1,     Stride2,  rzColumns,      rzRows,
                                points,  pointsStride1, Stride2,  pointsColumns,  pointsRows,
                                points2, pointsStride1, Stride2);
        if(returnValue!=0)
        {
            cout<<"something's wrong the multiplication of Ry by Points in 'place3dPoints'.\n";
            fflush(stdout);
            return true;
        }
    */
    


    //*****************************************
    // 2. Apply a vertical and horizontal shift
    //*****************************************
    //sum floorDistance to all the elements in the first row of "points2".
    _tempMat1.setTo(cv::Scalar(floorDistance)); //set all elements of _tempMat1 to the value of "floorDistance"
    cv::add(_p2Mat1,_tempMat1,_p2Mat1);         //_p2Mat1=_p2Mat1+_tempMat1.

    //used to be:
    //     ippsAddC_32f_I(floorDistance,points2,pointsColumns);
    //     if(returnValue!=0)
    //     {
    //         cout<<"something's wrong the shift in 'place3dPoints'.\n";
    //         fflush(stdout);
    //         return true;
    //     }


    //sum z to the third row of "points2".
    _tempMat3.setTo(cv::Scalar(z)); //set all elements of _tempMat3 to the value of "z"
    cv::add(_p2Mat3,_tempMat3,_p2Mat3);         //_p2Mat3=_p2Mat3+_tempMat3.

    //used to be:
    /*
    ippsAddC_32f_I(z,points2+2*pointsColumns,pointsColumns);
    if(returnValue!=0)
    {
        cout<<"something's wrong the shift in 'place3dPoints'.\n";
        fflush(stdout);
        return true;
    }
    */
    
    
    
    //**********************************
    //3. Rotate points around the Z axis
    //**********************************
    //Multiply RZ by "points2", put the result in "points"
    //points=_rzMat*_points2Mat     [3 x 2*nPixels]
    cv::multiply(_rzMat,_points2Mat,points);


    //used to be:
    /*
    returnValue=ippmMul_mm_32f(rz,      rzStride1,     Stride2,  rzColumns,      rzRows,
                               points2,  pointsStride1, Stride2,  pointsColumns,  pointsRows,
                               points, pointsStride1, Stride2);
    if(returnValue!=0)
    {
        cout<<"something's wrong the multiplication of Rz by Points2 in 'place3dPoints'.\n";
        fflush(stdout);
        return true;
    }
    */
    
    return false;
}


int PF3DTracker::perspective_projection(cv::Mat & xyz, double fx, double fy, double cx, double cy, cv::Mat & uv)
{
    //fill the projection matrix with the current values.
    ((double*)(_projectionMat.data + _projectionMat.step*0))[0]= fx;
    ((double*)(_projectionMat.data + _projectionMat.step*0))[1]=  0;
    ((double*)(_projectionMat.data + _projectionMat.step*0))[2]= cx;
    ((double*)(_projectionMat.data + _projectionMat.step*1))[0]=  0;
    ((double*)(_projectionMat.data + _projectionMat.step*1))[1]= fy;
    ((double*)(_projectionMat.data + _projectionMat.step*1))[2]= cy;

    //     int a,b;
    //     for(a=0;a<2;a++)
    //     {
    //         cout<<"LINE ";
    //         for(b=0;b<3;b++)
    //         {
    //             cout<<((double*)(_projectionMat.data + _projectionMat.step*a))[b]<<",";
    //         }
    //         cout<<"\n";
    //     }
    //     cout<<"\n";

    //     for(a=0;a<3;a++)
    //     {
    //         cout<<"LINE ";
    //         for(b=0;b<2*nPixels;b++)
    //         {
    //             cout<<((double*)(xyz.data + xyz.step*a))[b]<<",";
    //         }
    //         cout<<"\n";
    //     }
    //     cout<<"\n";

    //#####################################################
    //For every column of XYZ, divide each element by Z(i).
    //#####################################################
    //setup

    _xyzMat1=xyz(cv::Range(0,1), cv::Range::all());
    _xyzMat2=xyz(cv::Range(1,2), cv::Range::all());
    _xyzMat3=xyz(cv::Range(2,3), cv::Range::all());



    //divide X (the first line of xyz) by Z (the third line of xyz).
    cv::divide( _xyzMat1, _xyzMat3, _xyzMat1, 1 );

    //     for(a=0;a<3;a++)
    //     {
    //         cout<<"LINE ";
    //         for(b=0;b<2*nPixels;b++)
    //         {
    //             cout<<((double*)(xyz.data + xyz.step*a))[b]<<",";
    //         }
    //         cout<<"\n";
    //     }
    //     cout<<"\n";

    //divide Y (the second line of xyz) by Z (the third line of xyz).
    cv::divide( _xyzMat2, _xyzMat3, _xyzMat2, 1 );

    //     for(a=0;a<3;a++)
    //     {
    //         cout<<"LINE ";
    //         for(b=0;b<2*nPixels;b++)
    //         {
    //             cout<<((double*)(xyz.data + xyz.step*a))[b]<<",";
    //         }
    //         cout<<"\n";
    //     }
    //     cout<<"\n";

    //set all elements of Z to 1.
    _xyzMat3.setTo(cv::Scalar(1));

    //     for(a=0;a<3;a++)
    //     {
    //         cout<<"LINE ";
    //         for(r0;b<2*nPixels;b++)
    //         {
    //             cout<<((double*)(xyz.data + xyz.step*a))[b]<<",";
    //         }
    //         cout<<"\n";
    //     }
    //     cout<<"\n";

    //#########################
    //UV=projectionMat*(XYZ/Z).
    //#########################
    cvMatMul(_projectionMat,xyz,uv);

    /*    for(a=0;a<2;a++)
    {
        cout<<"LINE ";
        for(b=0;b<2*nPixels;b++)
        {
            cout<<((double*)(_uv.data + _uv.step*a))[b]<<",";
        }
        cout<<"\n";
    }
    cout<<"\n";*/
    return 0;
    
}        




bool PF3DTracker::computeHistogramFromRgbImage(cv::Mat & uv, cv::Mat & image,  CvMatND* innerHistogramMat, double &usedInnerPoints, CvMatND* outerHistogramMat, double &usedOuterPoints)
{
    //This should cross the R and B channels
    int count;
    int u,v;
    int r,g,b;
    int index;

    ////////
    //INNER/
    ////////
    usedInnerPoints=0;
    cvZero(innerHistogramMat);
    //used to be: ippsZero_32f(&innerHistogram[0][0][0], YBins*UBins*VBins);
    for(count=0;count<nPixels;count++)
    {
        u=(int)((double*)(uv.data + uv.step*0))[count]; //truncating ??? !!! warning
        v=(int)((double*)(uv.data + uv.step*1))[count]; //truncating ??? !!! warning
        if((v<image.rows)&&(v>=0)&&(u<image.cols)&&(u>=0))
        {


            //transform the color from RGB to HSI bin.
            r=(((uchar*)(image.data + image.step*v))[u*3+0]);
            g=(((uchar*)(image.data + image.step*v))[u*3+1]);
            b=(((uchar*)(image.data + image.step*v))[u*3+2]);
            index=r*65536+g*256+b;
            //increase the bin counter
            *((double*)(innerHistogramMat.data + _lut[index].y*innerHistogramMat->dim[0].step + _lut[index].u*innerHistogramMat->dim[1].step + _lut[index].v*innerHistogramMat->dim[2].step)) +=1;
            //used to be: innerHistogram[_lut[index].y][_lut[index].u][_lut[index].v]+=1; //increment the correct bin counter.
            usedInnerPoints+=1;
        }
    }

    //cout<<"inner points="<<usedInnerPoints<<endl;
    if(usedInnerPoints>0)
    {
        cvConvertScale( innerHistogramMat, innerHistogramMat, 1/usedInnerPoints, 0 );
        //used to be
        //ippsDivC_32f_I(usedInnerPoints, &innerHistogram[0][0][0], YBins*UBins*VBins);
    }

    ////////
    //OUTER/
    ////////
    usedOuterPoints=0;
    cvZero(outerHistogramMat);
    //used to be ippsZero_32f(&outerHistogram[0][0][0], YBins*UBins*VBins);
    for(count=nPixels;count<2*nPixels;count++)
    {
        u=(int)((double*)(uv.data + uv.step*0))[count]; //truncating ??? !!! warning
        v=(int)((double*)(uv.data + uv.step*1))[count]; //truncating ??? !!! warning
        if((v<image.rows)&&(v>=0)&&(u<image.cols)&&(u>=0))
        {
            //transform the color from RGB to HSI bin.
            r=(((uchar*)(image.data + image.step*v))[u*3+0]);
            g=(((uchar*)(image.data + image.step*v))[u*3+1]);
            b=(((uchar*)(image.data + image.step*v))[u*3+2]);
            index=r*65536+g*256+b;
            //increase the bin counter
            *((double*)(outerHistogramMat.data + _lut[index].y*outerHistogramMat->dim[0].step + _lut[index].u*outerHistogramMat->dim[1].step + _lut[index].v*outerHistogramMat->dim[2].step)) +=1;
            //used to be: outerHistogram[_lut[index].y][_lut[index].u][_lut[index].v]+=1; //increment the correct bin counter.
            usedOuterPoints+=1;
        }
    }
    if(usedOuterPoints>0)
    {
        cvConvertScale( outerHistogramMat, outerHistogramMat, 1/usedOuterPoints, 0 );
        //used to be: ippsDivC_32f_I(usedOuterPoints, &outerHistogram[0][0][0], YBins*UBins*VBins);
    }

    return false;
}


bool PF3DTracker::calculateLikelihood(CvMatND* templateHistogramMat, CvMatND* innerHistogramMat, CvMatND* outerHistogramMat, double inside_outside, double &likelihood)
{

    likelihood=0;
    int a,b,c;
    
    if(_staticImageTest)
    {
        cout<<"Inner histogram\n";
        for(a=0;a<YBins;a++)
        {
            for(b=0;b<UBins;b++)
            {
                for(c=0;c<VBins;c++)
                {
                    cout<<*((double*)(innerHistogramMat.data + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step))<<" ";
                }
                cout<< " A"<<endl;
            }
            cout<<endl;
        }

        cout<<"Outer histogram\n";
        for(a=0;a<YBins;a++)
        {
            for(b=0;b<UBins;b++)
            {
                for(c=0;c<VBins;c++)
                {
                    cout<<*((double*)(outerHistogramMat.data + a*outerHistogramMat->dim[0].step + b*outerHistogramMat->dim[1].step + c*outerHistogramMat->dim[2].step))<<" ";
                }
                cout<< " B"<<endl;
            }
            cout<<endl;
        }

        cout<<"Template histogram\n";
        for(a=0;a<YBins;a++)
        {
            for(b=0;b<UBins;b++)
            {
                for(c=0;c<VBins;c++)
                {
                    cout<<*((double*)(templateHistogramMat.data + a*templateHistogramMat->dim[0].step + b*templateHistogramMat->dim[1].step + c*templateHistogramMat->dim[2].step))<<" ";
                }
                cout<< " C"<<endl;
            }
            cout<<endl;
        }
    }
    
    /*    cout<<"injecting fake inner histogram\n";
    for(a=0;a<YBins;a++)
    {
        for(b=0;b<UBins;b++)
        {
            for(c=0;c<VBins;c++)
            {
                *((double*)(innerHistogramMat.data + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)) =
                *((double*)(templateHistogramMat.data + a*templateHistogramMat->dim[0].step + b*templateHistogramMat->dim[1].step + c*templateHistogramMat->dim[2].step));
             }
        }
    }

    cout<<"injecting fake outer histogram\n";
    for(a=0;a<YBins;a++)
    {
        for(b=0;b<UBins;b++)
        {
            for(c=0;c<VBins;c++)
            {
                *((double*)(outerHistogramMat.data + a*outerHistogramMat->dim[0].step + b*outerHistogramMat->dim[1].step + c*outerHistogramMat->dim[2].step)) = 0.0;
            }
        }
    }  */
    
    for(a=0;a<YBins;a++)
        for(b=0;b<UBins;b++)
            for(c=0;c<VBins;c++)
            {
                likelihood=likelihood + sqrt(
                            *((double*)(innerHistogramMat.data + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)) *
                        *((double*)(templateHistogramMat.data + a*templateHistogramMat->dim[0].step + b*templateHistogramMat->dim[1].step + c*templateHistogramMat->dim[2].step)))
                        - _insideOutsideDifferenceWeight*sqrt(
                            *((double*)(outerHistogramMat.data + a*outerHistogramMat->dim[0].step + b*outerHistogramMat->dim[1].step + c*outerHistogramMat->dim[2].step)) *
                        *((double*)(innerHistogramMat.data + a*innerHistogramMat->dim[0].step + b*innerHistogramMat->dim[1].step + c*innerHistogramMat->dim[2].step)));

                /*              used to be
                likelihood=likelihood + sqrt(
                innerHistogram[a][b][c]*
                templateHistogram[a][b][c])
                - _insideOutsideDifferenceWeight*sqrt(
                outerHistogram[a][b][c]
                *innerHistogram[a][b][c]);*/
            }
    if(_staticImageTest)
    {
        cout<<"initial likelihood = "<<likelihood<<endl;
    }
    likelihood=(likelihood+_insideOutsideDifferenceWeight)/(1+_insideOutsideDifferenceWeight);
    if(_staticImageTest)
    {
        cout<<"normalized likelihood = "<<likelihood<<endl;
    }
    if(likelihood<0)
        cout<<"LIKELIHOOD<0!!!\n";
    return false;
}




void printMat(cv::Mat & A)
{
    int a,b;
    for(a=0;a<A.rows;a++)
    {
        for(b=0;b<A.cols;b++)
        {
            cout<<((double*)(A.data + A.step*a))[b]<<",";
        }
        cout<<"\n";
    }
    
}


void PF3DTracker::drawSampledLinesPerspective(cv::Mat & model3dPointsMat, double x, double y, double z, cv::Mat & image,double _perspectiveFx,double  _perspectiveFy ,double _perspectiveCx,double  _perspectiveCy ,int R, int G, int B, double &meanU, double &meanV)
{


    bool failure;
    //cv::Mat & uv=cvCreateMat(2,2*nPixels,CV_32FC1);

    //create a copy of the 3D original points.
    model3dPointsMat.copyTo(_drawingMat);

    //used to be:
    //     Ipp32f model3dPointsDuplicate[3][2*nPixels];
    //     //printMatrix(&model3dPoints[0][0],2*nPixels,3);
    //     failure = ippsCopy_32f(&model3dPoints[0][0], &model3dPointsDuplicate[0][0],3*2*nPixels);
    //     //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);



    //****************************
    //ROTOTRANSLATE THE 3D POINTS.
    //****************************
    failure=place3dPointsPerspective(_drawingMat,x,y,z);
    //cout<<"rototraslated points:\n";
    //printMatrix(&model3dPointsDuplicate[0][0],2*nPixels,3);





    //***********************
    //PROJECT 3D POINTS TO 2D
    //***********************
    failure= perspective_projection(_drawingMat, _perspectiveFx, _perspectiveFy, _perspectiveCx, _perspectiveCy, _uv)!=0;
    if(failure)
    {
        cout<<"I had troubles projecting the points.\n";
    }
    //used to be:
    //         //(Ipp32f *xyz, int xyzColumns, int xyzRows, int xyzStride1, int xyzStride2, double fx, double fy, Ipp32f u0, Ipp32f v0, Ipp32f *uv, int uvStride1, int uvStride2
    //         failure=perspective_projection(&model3dPointsDuplicate[0][0],2*nPixels,3,sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f),_perspectiveFx,  _perspectiveFy , _perspectiveCx,  _perspectiveCy,&uv[0][0],sizeof(Ipp32f)*2*nPixels,sizeof(Ipp32f));
    //
    //         if(failure)
    //         {
    //             cout<<"I had troubles projecting the points.\n";
    //         }




    //DRAW
    int conta,uPosition,vPosition;
    meanU=0;
    meanV=0;
    for(conta=0;conta<nPixels;conta++)
    {
        meanU=meanU+((double*)(_uv.data + _uv.step*0))[conta];
        meanV=meanV+((double*)(_uv.data + _uv.step*1))[conta];


        vPosition= (int)((double*)(_uv.data + _uv.step*1))[conta];
        uPosition= (int)((double*)(_uv.data + _uv.step*0))[conta];
        if((uPosition<image.cols)&&(uPosition>=0)&&(vPosition<image.rows)&&(vPosition>=0))
        {
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+0])=R;
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+1])=G;
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+2])=B;
        }
        vPosition= (int)((double*)(_uv.data + _uv.step*1))[conta+nPixels];
        uPosition= (int)((double*)(_uv.data + _uv.step*0))[conta+nPixels];
        if((uPosition<image.cols)&&(uPosition>=0)&&(vPosition<image.rows)&&(vPosition>=0))
        {
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+0])=R;
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+1])=G;
            (((uchar*)(image.data + image.step*vPosition))[uPosition*3+2])=B;
        }

    }

    meanU=floor(meanU/nPixels);
    int meanUInt=(int)meanU;
    meanV=floor(meanV/nPixels);
    int meanVInt=(int)meanV;
    if((meanUInt<image.cols)&&(meanUInt>=0)&&(meanVInt<image.rows)&&(meanVInt>=0))
    {
        (((uchar*)(image.data + image.step*meanVInt))[meanUInt*3+0])=R;
        (((uchar*)(image.data + image.step*meanVInt))[meanUInt*3+1])=G;
        (((uchar*)(image.data + image.step*meanVInt))[meanUInt*3+2])=B;
    }


}


