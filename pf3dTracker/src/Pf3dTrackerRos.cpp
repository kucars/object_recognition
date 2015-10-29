#include "Pf3dTrackerRos.h"

Pf3dTrackerRos::Pf3dTrackerRos(const ros::NodeHandle & n_, const ros::NodeHandle & n_priv_) : n(n_), n_priv(n_priv_), it(n)
{
    int nParticles;
    double accelStdDev;
    double insideOutsideDifferenceWeight;
    double likelihoodThreshold;
    std::string models_folder;
    std::string trackedObjectColorTemplate;
    std::string trackedObjectColorTemplateFile;

    std::string trackedObjectShapeTemplate;
    std::string trackedObjectShapeTemplateFile;

    std::string motionModelMatrix;
    std::string motionModelMatrixFile;

    std::string dataFileName;
    std::string initializationMethod;
    double initialX;
    double initialY;
    double initialZ;

    int calibrationImageWidth;
    int calibrationImageHeight;
    double perspectiveFx;
    double perspectiveFy;
    double perspectiveCx;
    double perspectiveCy;
    //***********************************************
    // Read parameters from the initialization file *
    //***********************************************
    //Topics

    //Parameters for the algorithm
    n_priv.param<int>("nParticles", nParticles, 900); //TODO set it back to 900
    n_priv.param<double>("accelStdDev", accelStdDev, 30.0); //TODO set it back to 30
    n_priv.param<double>("insideOutsideDiffWeight", insideOutsideDifferenceWeight, 1.5);
    n_priv.param<double>("likelihoodThreshold", likelihoodThreshold, 0.005);//.005); TODO ser it back to 0.005
    n_priv.param<std::string>("models_folder", models_folder, "models_folder");
    n_priv.param<std::string>("trackedObjectColorTemplate", trackedObjectColorTemplate, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/red_ball_iit.bmp");
    n_priv.param<std::string>("trackedObjectShapeTemplate", trackedObjectShapeTemplate, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/initial_ball_points_smiley_31mm_20percent.csv");
    n_priv.param<std::string>("motionModelMatrix", motionModelMatrix, "/home/vislab/repositories/ros/object_recognition/pf3dTracker/models/motion_model_matrix.csv");
    n_priv.param<std::string>("trackedObjectTemp", dataFileName, "current_histogram.csv"); //might be removed? TODO
    n_priv.param<std::string>("initializationMethod", initializationMethod, "3dEstimate");
    n_priv.param<double>("initialX", initialX, 0.0);
    n_priv.param<double>("initialY", initialY, 0.0);
    n_priv.param<double>("initialZ", initialZ, 0.2);
    n_priv.param<int>("w", calibrationImageWidth,        320); //NOT USED AT THE MOMENT
    n_priv.param<int>("h", calibrationImageHeight,       240);  //NOT USED AT THE MOMENT
    n_priv.param<double>("perspectiveFx", perspectiveFx, 980.0);
    n_priv.param<double>("perspectiveFy", perspectiveFy, 982.0);
    n_priv.param<double>("perspectiveCx", perspectiveCx, 320.0);
    n_priv.param<double>("perspectiveCy", perspectiveCy, 240.0);
    //Units are meters outside the program, but millimeters inside, so we need to convert
    initialX*=1000.0;
    initialY*=1000.0;
    initialZ*=1000.0;

    //Camera intrinsic parameters


    bool _staticImageTest = false;

    if(_staticImageTest)
    {
        nParticles=1;
        accelStdDev=0.0001;
        initialX =  -45.0; //careful: these are millimeters!
        initialY =  -45.0; //careful: these are millimeters!
        initialZ =  290.0; //careful: these are millimeters!
    }

    trackedObjectColorTemplateFile=models_folder+trackedObjectColorTemplate;
    trackedObjectShapeTemplateFile=models_folder+trackedObjectShapeTemplate;
    motionModelMatrixFile=models_folder+motionModelMatrix;

    ROS_INFO_STREAM("nParticles: "<<nParticles);
    ROS_INFO_STREAM("accelStdDev: "<<accelStdDev);
    ROS_INFO_STREAM("insideOutsideDifferenceWeight: "<<insideOutsideDifferenceWeight);
    ROS_INFO_STREAM("likelihoodThreshold: "<<likelihoodThreshold);
    ROS_INFO_STREAM("trackedObjectColorTemplate: "<<trackedObjectColorTemplate);
    ROS_INFO_STREAM("trackedObjectShapeTemplate: "<<trackedObjectShapeTemplate);
    ROS_INFO_STREAM("motionModelMatrix: "<<motionModelMatrix);
    ROS_INFO_STREAM("initializationMethod: "<<initializationMethod);
    ROS_INFO_STREAM("initialX: "<<initialX);
    ROS_INFO_STREAM("initialY: "<<initialY);
    ROS_INFO_STREAM("initialZ: "<<initialZ);
    ROS_INFO_STREAM("calibrationImageWidth: "<<calibrationImageWidth);
    ROS_INFO_STREAM("calibrationImageHeight: "<<calibrationImageHeight);
    ROS_INFO_STREAM("perspectiveFx: "<<perspectiveFx);
    ROS_INFO_STREAM("perspectiveFy: "<<perspectiveFy);
    ROS_INFO_STREAM("perspectiveCx: "<<perspectiveCx);
    ROS_INFO_STREAM("perspectiveCy: "<<perspectiveCy);

    tracker=boost::shared_ptr<PF3DTracker>(new PF3DTracker(nParticles,
                                                           accelStdDev,
                                                           insideOutsideDifferenceWeight,
                                                           likelihoodThreshold,
                                                           trackedObjectColorTemplateFile,
                                                           trackedObjectShapeTemplateFile,
                                                           motionModelMatrixFile,
                                                           dataFileName,
                                                           initializationMethod,
                                                           initialX,
                                                           initialY,
                                                           initialZ,
                                                           calibrationImageWidth,
                                                           calibrationImageHeight,
                                                           perspectiveFx,
                                                           perspectiveFy,
                                                           perspectiveCx,
                                                           perspectiveCy));


    // Set the input video port, with the associated callback method
    image_in = it.subscribe("image_in", 1, &Pf3dTrackerRos::processImageCallback, this);

    // Set the output ports
    image_out = it.advertise("image_out", 1);

    // Set the estimates out topic
    estimates_out  = n.advertise<pf3d_tracker::Estimates>("estimates_out", 1);

}
void Pf3dTrackerRos::processImageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
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
    cv::Mat _rawImage;

    seed=rand();
    bool _staticImageTest=false;
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

    cv::Mat resized_image;
    cv::resize(_rawImage, resized_image, cv::Size(320, 240), 0, 0, cv::INTER_CUBIC); // resize to 1024x768 resolution

    tracker->processImage(resized_image);


    /////////////////
    // DATA OUTPUT //
    /////////////////

    /*pf3d_tracker::Estimates outMsg;
    outMsg.mean.point.x=tracker->weightedMeanX/1000;
    outMsg.mean.point.y=weightedMeanY/1000;
    outMsg.mean.point.z=weightedMeanZ/1000;

    outMsg.likelihood=tracker->maxLikelihood/exp((double)20.0);
    outMsg.meanU=meanU;
    outMsg.meanV=meanV;
    outMsg.seeingBall=_seeingObject;
    _outputDataPort.publish(outMsg);*/



    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_image).toImageMsg();

    image_out.publish(img_msg);

    if(_staticImageTest)
    {
        cv::imwrite("testImageOutput.png", _rawImage);
        ros::shutdown(); //exit the program after just one cycle
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pf3d_tracker"); // set up ROS
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    Pf3dTrackerRos tracker(n,n_priv); //instantiate the tracker.
    ros::spin(); // pass control on to ROS

    return 0;
}
