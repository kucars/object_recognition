/**
* Copyright: (C) 2009 RobotCub Consortium
* Authors: Matteo Taiana
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

//good combinations:
// _nParticles | stDev | inside_outside | nPixels
//	800       100	       1.5	    50
//with 800 particles and 30fps the 100% of the processor of CORTEX1 is used.
//could try to halve the number of pixels used for the circles.

#include <iostream>
#include <string>
#include <sstream>


#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
//#include "cv.h"
//#include "highgui.h"
#endif
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iCub/pf3dTrackerSupport.hpp>

//for tracking in the iCub: 1000 particles and an stDev of 80 work well with slow movements of the ball. the localization is quite stable. the shape model has a 20% difference wrt the real radius.
//#define _nParticles 5000
//#define stDev 100 80
//150 for the demo.
//#define stDev 150 


//#define NEWPOLICY 0
//1: only transform the color of the pixels that you need
//0: transform the whole image.

#define nPixels 50

#define YBins 4
#define UBins 8 
#define VBins 8

//should be 1.5 or 1.0 !!! ???
//#define inside_outside_difference_weight 1.5
//if I set it to 1.5, when the ball goes towards the camera, the tracker lags behind.
//this happens because the particles with "ball color" on both sides of the contour
//are still quite likely. the opposite is not true: when the ball goes away from the
// camera, the tracker follows it quite readily.

using namespace std;


class PF3DTracker
{

private:
    std::string dataFileName;
    std::string trackedObjectShapeTemplate;
    std::string trackedObjectColorTemplate;
    std::string motionModelMatrix;
    int _numParticlesReceived;


    bool supplyUVdata;

    string _projectionModel;
    double _perspectiveFx;
    double _perspectiveFy;
    double _perspectiveCx;
    double _perspectiveCy;
    int _calibrationImageWidth;
    int _calibrationImageHeight;
    double _likelihoodThreshold;

    int _seeingObject; //0 means false, 1 means true.
    int _circleVisualizationMode;
    string _initializationMethod;
    string _trackedObjectType;
    bool _saveImagesWithOpencv;
    string _saveImagesWithOpencvDir;
    double _initialX;
    double _initialY;
    double _initialZ;
    double _attentionOutput;
    double _attentionOutputMax;
    double _attentionOutputDecrease;
    cv::RNG rngState; //something needed by the random number generator
    bool _doneInitializing;

    Lut*   _lut;
    cv::Mat _A;
    int _nParticles;
    double accelStdDev;
    double _insideOutsideDifferenceWeight;
    int _colorTransfPolicy;

    //double _modelHistogram[YBins][UBins][VBins]; //data
    CvMatND* _modelHistogramMat; //OpenCV Matrix
    CvMatND* _innerHistogramMat;//[YBins][UBins][VBins];
    CvMatND* _outerHistogramMat;//[YBins][UBins][VBins];

    cv::Mat _model3dPointsMat; //shape model
    cv::Mat _visualization3dPointsMat; //visualization model for the sphere (when _circleVisualizationMode==1). should have less points, but it was easier to make it like this.


    //not sure which of these are really used.
    cv::Mat _rzMat;      //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _ryMat;      //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _points2Mat; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _p2Mat1;     //pointer to the first row of _points2Mat
    cv::Mat _p2Mat3;     //pointer to the third row of _points2Mat
    cv::Mat _tempMat;    //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _tempMat1;   //pointer to the first row of _tempMat
    cv::Mat _tempMat2;   //pointer to the first row of _tempMat
    cv::Mat _tempMat3;   //pointer to the first row of _tempMat
    cv::Mat _drawingMat; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _projectionMat; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _xyzMat1; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _xyzMat2; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _xyzMat3; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles1; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles2; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles3; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles4; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles5; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles6; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles7; //this is used during some operations... it is defined here to avoid repeated instantiations
    cv::Mat _particles1to6;
    cv::Mat _newParticles1to6;
    cv::Mat _uv;

    //resampling-related stuff
    cv::Mat _nChildren;
    cv::Mat _label;
    cv::Mat _u;
    cv::Mat _ramp;

    //new resampling-related stuff
    cv::Mat _cumWeight;

    //variables
    cv::Mat _particles;
    cv::Mat _newParticles;
    cv::Mat _noise;
    cv::Mat _noise1; //lines from 0 to 2
    cv::Mat _noise2; //lines from 3 to 5

    //sensor_msgs::CvBridge bridge_;
    cv::Mat frame;

    cv::Mat _rawImage;
    cv::Mat _transformedImage;//_yuvBinsImage[image_width][image_height][3];
    double _initialTime;
    double _finalTime;

    int _frameCounter;
    int _framesNotTracking; //number of frames for which the likelihood has been under the threshold. after 20 such frames the tracker is restarted.
    int downsampler;
    double _lastU;
    double _lastV;
    bool _firstFrame; //when processing the first frame do not write the fps values, as it's wrong.
    bool _staticImageTest; //when true, only one image is read from a file


    ///////////////////////
    //MEMBERS THAT "WORK":/
    ///////////////////////
    bool testOpenCv();
    bool readModelHistogram(CvMatND* histogram,const char fileName[]);
    bool readInitialmodel3dPoints(cv::Mat & points,string fileName);
    bool readMotionModelMatrix(cv::Mat & points, string fileName);
    bool computeTemplateHistogram(string imageFileName,string dataFileName); //I checked the output, it seems to work, but it seems as if in the old version the normalization didn't take effect.
    //bool computeHistogram(cv::Mat & uv, IplImage* transformedImage,  CvMatND* innerHistogramMat, double &usedInnerPoints, CvMatND* outerHistogramMat, double &usedOuterPoints);
    bool computeHistogramFromRgbImage(cv::Mat & uv, cv::Mat & image,  CvMatND* innerHistogramMat, double &usedInnerPoints, CvMatND* outerHistogramMat, double &usedOuterPoints);
    bool calculateLikelihood(CvMatND* templateHistogramMat, CvMatND* innerHistogramMat, CvMatND* outerHistogramMat, double inside_outside, double &likelihood);
    bool place3dPointsPerspective(cv::Mat & points, double x, double y, double z);
    int perspective_projection(cv::Mat & xyz, double fx, double fy, double cx, double cy, cv::Mat & uv);
    void drawContourPerspective(cv::Mat & model3dPointsMat,double x, double y, double z, cv::Mat & image, double _perspectiveFx,double  _perspectiveFy ,double _perspectiveCx,double  _perspectiveCy ,int R, int G, int B, double &meanU, double &meanV);
    void drawSampledLinesPerspective(cv::Mat & model3dPointsMat, double x, double y, double z, cv::Mat & image,double _perspectiveFx,double  _perspectiveFy ,double _perspectiveCx,double  _perspectiveCy ,int R, int G, int B, double &meanU, double &meanV);

    //bool evaluateHypothesisPerspective(cv::Mat & model3dPointsMat, double x, double y, double z, CvMatND* modelHistogramMat, IplImage* transformedImage, double fx, double fy, double u0, double v0, double, double &likelihood);




    //////////////////////////////////////////////
    //MEMBERS THAT SHOULD BE CHANGED AND CHECKED:/
    //////////////////////////////////////////////
    bool evaluateHypothesisPerspectiveFromRgbImage(cv::Mat & model3dPoints,double x, double y, double z, CvMatND* modelHistogramMat, cv::Mat & image,  double fx, double fy, double u0, double v0, double inside_outside, double &likelihood);

    bool systematicR(cv::Mat & inState, cv::Mat & weights, cv::Mat & outState);
    bool systematic_resampling(cv::Mat & oldParticlesState, cv::Mat & oldParticlesWeights, cv::Mat & newParticlesState, cv::Mat & cumWeight);

public:
    PF3DTracker(const int & __nParticles,
                const double & __accelStdDev,
                const double & __insideOutsideDifferenceWeight,
                const double & __likelihoodThreshold,
                const std::string & __trackedObjectColorTemplate,
                const std::string & __trackedObjectShapeTemplate,
                const std::string & __motionModelMatrix,
                const std::string & __dataFileName,
                const std::string & __initializationMethod,
                const double & __initialX,
                const double & __initialY,
                const double & __initialZ,
                const int & __calibrationImageWidth,
                const int & __calibrationImageHeight,
                const double & __perspectiveFx,
                const double & __perspectiveFy,
                const double & __perspectiveCx,
                const double & __perspectiveCy); //constructor
    ~PF3DTracker(); //destructor

    bool open(); //member to set the object up.
    virtual bool close();                  //member to close the object.
    void processImage(const cv::Mat & image);

    //virtual bool updateModule();           //member that is repeatedly called by YARP, to give this object a chance to do something.

};


