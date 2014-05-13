#! /usr/bin/env python

import roslib; #roslib.load_manifest('tabletop_segmentation_online')
import rospy
#roslib.load_manifest('ist_generate_grasps')
import actionlib
#roslib.load_manifest('ist_tasks')
import perception_msgs.msg
import std_srvs.srv
from tabletop_object_segmentation_online.srv import *
from perception_msgs.srv import *
import tf
import math
import time
#from wsg_gripper.msg import *


class DetectObjectsAction(object):
  # create messages that are used to publish feedback/result
  _feedback = perception_msgs.msg.DetectObjectsFeedback()
  _result   = perception_msgs.msg.DetectObjectsResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, perception_msgs.msg.DetectObjectsAction, execute_cb=self.execute_cb)
    self._as.start()

  def execute_cb(self, goal):

    rospy.set_param("/tabletop_segmentation/x_filter_max",  goal.table_region.x_filter_max)
    rospy.set_param("/tabletop_segmentation/x_filter_min",  goal.table_region.x_filter_min)
    rospy.set_param("/tabletop_segmentation/y_filter_max",  goal.table_region.y_filter_max)
    rospy.set_param("/tabletop_segmentation/y_filter_min",  goal.table_region.y_filter_min)
    rospy.set_param("/tabletop_segmentation/z_filter_max",  goal.table_region.z_filter_max)
    rospy.set_param("/tabletop_segmentation/z_filter_min",  goal.table_region.z_filter_min)
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
        
    ####################################
    # start executing the action steps #
    ####################################

    object_list=self.execution_steps(goal)
    if object_list:
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
    else:
        self._as.set_succeeded(self._result)

  def tabletop_segmentation(self):
    print 'waiting for tabletop segmentation service...'
    rospy.wait_for_service('tabletop_segmentation')

    try:
      table_top = rospy.ServiceProxy('tabletop_segmentation' , TabletopSegmentation)
      resp = table_top()
      if len(resp.clusters) == 0:
        print 'No clusters found'
        return False
      else:
	print 'found ' + str(len(resp.clusters)) + ' clusters!'
        self._result.table=resp.table
        return resp
    except rospy.ServiceException, e:
        print "TabletopSegmentation Service call failed: %s"%e
        return False

  def object_recognition_and_pose_estimation(self,segmentation_resp):
    print 'waiting for object recognition and pose estimation service...'
    rospy.wait_for_service('object_recognition_pose_estimation')
    try:
      obj_rec_pose_est = rospy.ServiceProxy('object_recognition_pose_estimation' , PoseEstimation)
      myReq = PoseEstimationRequest()
      myReq.table=segmentation_resp.table
      myReq.cluster_list=segmentation_resp.clusters
      resp = obj_rec_pose_est(myReq)
      if len(resp.object_list.graspable_objects) == 0:
        print 'No objects found'
        return False
      else:
        self._result.object_list=resp.object_list
        return resp
    except rospy.ServiceException, e:
        print "Object recognition pose estimation Service call failed: %s"%e
        return False

  def object_details(self,complete_shapes):    
    print 'waiting for object details service...'
    rospy.wait_for_service('ist_compute_object_details')
    object_list = ist_msgs.msg.ObjectList()
    try:
        object_details = rospy.ServiceProxy('ist_compute_object_details' , perception_msgs.srv.GetObjectDetails)
        for i in range(0,len(complete_shapes.objects)):
          myReqDet = perception_msgs.srv.GetObjectDetailsRequest()
          myReqDet.point_cloud = complete_shapes.objects[i].point_cloud
          #myReqDet.object_name = std_msgs.msg.String(str(object_name))
          myReqDet.point_cloud_object_details = complete_shapes.objects[i].point_cloud_object_details
          obj_det_resp = object_details(myReqDet)
          #raw_input("Waiting for a keystroke")
          object_list.objects.append(obj_det_resp.object)
    except rospy.ServiceException, e:
        print "GetObjectDetails Service call failed: %s"%e
        return False
    return object_list


        
  def execution_steps(self,goal):
       
    # 1. Segmentation

    # publish the feedback
    self._feedback.state="Executing tabletop segmentation..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)

    # Service call
    segmentation_resp=self.tabletop_segmentation()
    if segmentation_resp==False:
        self._as.set_aborted(self._result)
        return False

    self._result.table = segmentation_resp.table
    self._result.clusters = segmentation_resp.clusters
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
        
    # publish the feedback
    self._feedback.state="Done." 
    self._feedback.progress=50.0
    self._as.publish_feedback(self._feedback)

    # 2. Object recognition and pose estimation

    # publish the feedback
    self._feedback.state="Executing object recognition and pose estimation..." 
    self._feedback.progress=51.0
    self._as.publish_feedback(self._feedback)

    # Service call
    object_recognition_and_pose_estimation_resp=self.object_recognition_and_pose_estimation(segmentation_resp)
    if object_recognition_and_pose_estimation_resp==False:
        self._as.set_aborted(self._result)
        return False

    self._result.object_list = object_recognition_and_pose_estimation_resp.object_list

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False

    # publish the feedback
    self._feedback.state="Done." 
    self._feedback.progress=75.0
    self._as.publish_feedback(self._feedback)


     # 3. Object details

    # publish the feedback
    self._feedback.state="Computing object details." 
    self._feedback.progress=76.0
    self._as.publish_feedback(self._feedback)
   
    # Service call
    object_list=self.object_details(shape_completion_resp)
    if object_list==False:
        #self._as.set_aborted(self._result)
        return False
    print_objects_info(object_list)  
     
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
        
    # publish the feedback
    self._feedback.state="Done." 
    self._feedback.progress=100.0
    self._as.publish_feedback(self._feedback)
    
    return object_recognition_and_pose_estimation_resp
    

  def request_base_link_to_table_tf(self):
    listener = tf.TransformListener()
    listener.waitForTransform('/base_link','/table_frame',rospy.Time(),rospy.Duration(2.0))
    try:
      (trans,rot) = listener.lookupTransform('/base_link','/table_frame',rospy.Time(0))
      print 'Translation component' + str(trans)
      print 'Rotation component' + str(rot)
      #base_table_quaternion_array=[rot.x,rot.y,rot.z,rot.w]
      #hand_object_translation_array=[hand_object_pose.position.x,hand_object_pose.position.y,hand_object_pose.position.z]
      #hand_object_hmatrix=numpy.mat(tf.transformations.quaternion_matrix(hand_object_quaternion_array)+tf.transformations.translation_matrix(hand_object_translation_array)-tf.transformations.identity_matrix())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print "Transform exception"
    return


if __name__ == '__main__':
  rospy.init_node('detect_objects_server')
  DetectObjectsAction(rospy.get_name())
  rospy.spin()

