/** 
\defgroup icub_pf3dTracker pf3dTracker
@ingroup icub_module   

A robust and real-time 3D ball tracker.

Copyright (C) 2009 RobotCub Consortium
 
Author: Matteo Taiana, <A HREF="http://users.isr.ist.utl.pt/~mtaiana/">homepage</A>.

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module implements a model-based object tracker: it estimates the 3D position of a ball given a sequence of images. One estimate is computed for each input image. The tracker outputs a flag that is set to 1 when the module is confident that it is tracking a ball, when the flag is set to 0 the tracker still computes a 3D estimate, but that is meaningless. After 5 frames with confidence under the threshold, the tracker estimate is reset.
The module is meant to work together with \ref icub_pf3dBottomup "pf3dBottomup", which is a 3D ball detector, but can also work on its own.

<b>For an explanation on how to configure the tracker and bottom up modules, how to connect them, how to run them through the application manager, etc., please have a look at <A HREF="http://mediawiki.isr.ist.utl.pt/wiki/3D_ball_tracker">this page</A>.</b>

<b>The algorithm is described in the paper: "Tracking objects with generic calibrated sensors: an algorithm based on color and 3D shape features", please cite it if you use the tracker in your research.</b>

You can watch a video of the tracker in action <A HREF="http://www.youtube.com/watch?v=Xp8qUhfMzhU">here</A>.

\section lib_sec Libraries 
- YARP libraries. 
- OpenCv
 
\section parameters_sec Parameters
Configuration is done through an initialization file (See for instance $ICUB_ROOT/main/app/pf3dTracker/conf/pf3dTracker.ini) Here is an example, with comments: 
 \code 
 ####################################
 #configuration file for pf3dTracker#
 ####################################
 
 
 #############
 #module name#
 #############
 name                        /pf3dTracker
 
 #############################
 #parameters of the algorithm#
 #############################
 nParticles                  900
 #nParticles                 number of particles used
 accelStDev                  30
 #accelStDev                 standard deviation of the acceleration noise
 insideOutsideDiffWeight     1.5
 #insideOutsideDiffWeight    inside-outside difference weight for the likelihood function
 colorTransfPolicy           1
 #colorTransfPolicy          [0=transform the whole image | 1=only transform the pixels you need]
 
 
 #########################
 #port names and function#
 #########################
 inputVideoPort              /pf3dTracker/video:i
 #inputVideoPort             receives images from the grabber or the rectifying program.
 outputVideoPort             /pf3dTracker/video:o
 #outputVideoPort            produces images in which the contour of the estimated ball is highlighted.
 outputDataPort              /pf3dTracker/data:o
 #outputDataPort             produces a stream of data in the format: X, Y, Z [meters], likelihood, U, V [pixels], seeing_object.
 inputParticlePort           /pf3dTracker/particles:i
 #inputParticlePort          receives hypotheses on the position of the ball from the bottom up module
 outputParticlePort          /pf3dTracker/particles:o
 #outputParticlePort         produces data for the plotter. it is usually not active for performance reasons.
 outputAttentionPort         /pf3dTracker/attention:o
 #outputAttentionPort        produces data for the attention system, in terms of a peak of saliency.
 
 
 #################################
 #projection model and parameters#
 #################################
 #projectionModel, only the perspective one was implemented so far.
 projectionModel             perspective
 
 #iCubLisboaLeftEye_Zoom_Lens_2009_05_19
 w 320
 h 240
 perspectiveFx 445.202
 perspectiveFy 445.664
 perspectiveCx 188.297
 perspectiveCy 138.496
 
 
 #######################
 #tracked object models#
 #######################
 #trackedObjectType, only sphere was implemented so far.
 trackedObjectType           sphere
 trackedObjectColorTemplate  models/red_smiley_2009_07_02.bmp
 trackedObjectShapeTemplate  models/initial_ball_points_smiley_31mm_20percent.csv
 
 motionModelMatrix           models/motion_model_matrix.csv
 trackedObjectTemp           current_histogram.csv
 
 
 #######################
 #initialization method#
 #######################
 #initialization method, only 3dEstimate was implemented so far.
 initializationMethod        3dEstimate
 #initial position [meters]
 initialX                       0
 initialY                       0
 initialZ                       0.5  
 
 
 ####################
 #visualization mode#
 ####################
 #circleVisualizationMode [0=inner and outer circle | 1=one circle with the correct radius]
 #default 0. only applies to the sphere.
 circleVisualizationMode 1
 
 
 #################################
 #likelihood and reset condition #
 #################################
 #the tracker produces a value of likelihood at each time step.
 #this value can be used to infer if the object it is tracking is the correct one.
 #
 #if likelihood<=this value for 5 consecutive frames, the tracker
 #assumes it's not seeing the right object and is reinitialized.
 #
 likelihoodThreshold         0.005

\endcode 

\section portsa_sec Ports Accessed
The tracker need to be connected to a port that streams images, at the very least, in order to work.
 
\section portsc_sec Ports Created 
- /pf3dTracker/video:i receives the image stream given which the ball has to be tracked.

- /pf3dTracker/video:o produces images in which the contour of the estimated ball is highlighted. When the tracker is confident that it's tracking a ball, it draws the contour in green, when it is not confident (it's looking for a ball, but does not yet have a good estimate), it draws the contour in yellow.

- /pf3dTracker/data:o produces a stream of data in the format: X, Y, Z [meters], likelihood, U, V [pixels], seeing_object. <br>
X, Y and Z are the estimated coordinates of the tracked ball in the eye reference frame (they can be transformed to the root reference frame by module \ref eye2RootFrameTransformer "eye2RootFrameTransformer". The likelihood value indicates how confident the tracker is that the object it's tracking is the right ball (the lower the likelihood, the lower the confidence, but beware that even a perfect match will result in a value pretty far from 1). U and V are the estimated coordinates of the centre of the ball in the image plane, U is horizontal and V vertical, the origin is on the top left corner of the image. Seeing_object is a flag, it is set 1 when the likelihood is higher than a threshold specified in the initialization file, it is set to 0 otherwise. When the tracker experiences 5 consecutive images with seeing_object==0, the estimate is reset. This prevents the tracker from getting stuck on an unlikely target.

- /pf3dTracker/particles:i receives hypotheses on 3D poses of a ball, normally produced by the \ref icub_pf3dBottomup "pf3dBottomup" detection module. If the tracker does not receive anything on this port, it behaves normally, i.e., it needs more time to find a ball and start tracking it, after initialization.

- /pf3dTracker/particles:o produces data for the plotter. it is usually not active for performance reasons.

- /pf3dTracker/attention:o produces data for the attention system, in terms of a peak of saliency.
 

 
\section in_files_sec Input Data Files
The tracker requires three input files: an image file used to build the colour model of the tracked ball, a file defining the motion model applied to the estimate of the ball between frames, and a shape model for the ball, defining its size and the distance between inner and outer contours (see the paper cited in the description section). For more detail on these files, please have a look at the parameters section.

\section out_data_sec Output Data Files 
The module produces one output file that is only useful for debugging (current_histogram.csv) and can be configured to save the images it produces to files, but this is not recommended: use the dataDumper module instead.

\section tested_os_sec Tested OS
Ubuntu Linux, Windows

\author Matteo Taiana, <A HREF="http://users.isr.ist.utl.pt/~mtaiana/">homepage</A>.
*/ 

#include <ros/ros.h>
#include <iCub/pf3dTracker.hpp>

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pf3dTracker"); // set up ROS
	ros::NodeHandle n;
 	PF3DTracker tracker(n); //instantiate the tracker.
	ros::spin(); // pass control on to ROS

	return 0;
}


