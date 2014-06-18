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




def readable_object_discrete_pose(discrete_pose):
    if(discrete_pose==1):
        return '  pose: vertical     id: ' + str(discrete_pose)
    elif(discrete_pose==2):
        return '  pose: horizontal   id: ' + str(discrete_pose) 
    else:
        return '  pose: any          id: ' + str(discrete_pose) 


def readable_object_part_task_info(object_task):
    if(object_task.id==1):
        return  'id: ' + str(object_task.id) + '  name: pour out                    ' + '  likelihood: ' + str(object_task.likelihood)
    elif(object_task.id==2):
        return  'id: ' + str(object_task.id) + '  name: pass                        ' + '  likelihood: ' + str(object_task.likelihood) 
    elif(object_task.id==3):
        return  'id: ' + str(object_task.id) + '  name: pour in                     ' + '  likelihood: ' + str(object_task.likelihood) 
    elif(object_task.id==4):
        return  'id: ' + str(object_task.id) + '  name: pick place inside upsidedown' + '  likelihood: ' + str(object_task.likelihood)
    elif(object_task.id==5):
        return  'id: ' + str(object_task.id) + '  name: pick place inside upright   ' + '  likelihood: ' + str(object_task.likelihood)
    elif(object_task.id==6):
        return  'id: ' + str(object_task.id) + '  name: pick place inside sideways  ' + '  likelihood: ' + str(object_task.likelihood)
    elif(object_task.id==7):
        return  'id: ' + str(object_task.id) + '  name: pick place on               ' + '  likelihood: ' + str(object_task.likelihood)
    else:
        return 'wrong id'

def readable_object_part_tasks_info(object_tasks):
    temp='tasks: '
    for task_list_index in range(0,len(object_tasks)):
        temp=temp + '\n    ' + readable_object_part_task_info(object_tasks[task_list_index])
    return temp

def readable_object_part_info(object_part):
    if(object_part.part.id==1):
        return  'id: ' + str(object_part.part.id) + '  name: top' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)
    elif(object_part.part.id==2):
        return  'id: ' + str(object_part.part.id) + '  name: middle' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)
    elif(object_part.part.id==3):
        return  'id: ' + str(object_part.part.id) + '  name: bottom' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)
    elif(object_part.part.id==4):
        return  'id: ' + str(object_part.part.id) + '  name: handle' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)
    elif(object_part.part.id==5):
        return  'id: ' + str(object_part.part.id) + '  name: usable' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)
    else:
        return  'id: ' + str(object_part.part.id) + '  name: any   ' + '  confidence: ' +  str(object_part.part.confidence) + '\n   ' +  readable_object_part_tasks_info(object_part.tasks)

def readable_object_parts_info(object_parts):
    temp='parts: '
    for part_list_index in range(0,len(object_parts)):
        temp=temp + '\n   ' + readable_object_part_info(object_parts[part_list_index])
    return temp

def readable_object_category_info(object_category):
    return  'id: ' + str(object_category.id) + '  name: ' + str(object_category.name) + '  likelihood: ' + str(object_category.likelihood)


def readable_object_categories_info(object_categories):
    temp='categories: '
    for category_list_index in range(0,len(object_categories)):
        temp=temp + '\n    ' + readable_object_category_info(object_categories[category_list_index])
    return temp

def readable_object_type_info(object_type):
    return 'type:\n   id:  ' + str(object_type.id)  + '  type name: ' +  str(object_type.type_name) + '  is container: ' +  str(object_type.is_container)


def readable_object_info(id,object):
    return 'object '+str(object.object_id+1)+': \n  '+readable_object_discrete_pose(object.state.discrete_pose)+ '\n  ' + readable_object_type_info(object.data.type) + '\n  ' + readable_object_categories_info(object.data.category_hypotheses) + '\n  ' + readable_object_parts_info(object.data.actionable_parts_data)

def print_objects_info(object_list):

    for object_list_index in range(0,len(object_list.objects)):
        print 'Object info: \n ' + readable_object_info(object_list_index,object_list.objects[object_list_index])


	





class DetectObjectsAction(object):
  # create messages that are used to publish feedback/result
  _feedback = perception_msgs.msg.DetectObjectsFeedback()
  _result   = perception_msgs.msg.DetectObjectsResult()


  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, perception_msgs.msg.DetectObjectsAction, execute_cb=self.execute_cb)
    self._as.start()
    self._first=True

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
        self._result.object_list=object_list
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
	return True
    else: 
        return False
    #else:
    #    rospy.loginfo('%s: Failed' % self._action_name)
    #    self._as.set_aborted(self._result)

  def tabletop_segmentation(self, table):
    print 'waiting for tabletop segmentation service...'
    rospy.wait_for_service('tabletop_segmentation')

    try:
      table_top = rospy.ServiceProxy('tabletop_segmentation' , TabletopSegmentation)

      myReq = TabletopSegmentationRequest()
      if self._first==False:
        myReq.table=table

      resp = table_top(myReq)   

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
        #self._result.object_list=resp.object_list
        return resp
    except rospy.ServiceException, e:
        print "Object recognition pose estimation Service call failed: %s"%e
        return False

  def object_details(self,object_list):    
    print 'waiting for object details service...'
    rospy.wait_for_service('ist_compute_object_details')
    ist_object_list = ist_msgs.msg.ObjectList()
    try:
        object_details = rospy.ServiceProxy('ist_compute_object_details' , perception_msgs.srv.GetObjectDetails)
        for i in range(0,len(object_list.graspable_objects)):
          myReqDet = perception_msgs.srv.GetObjectDetailsRequest()
          myReqDet.point_cloud = object_list.graspable_objects[i].region.cloud
          #myReqDet.object_name = std_msgs.msg.String(str(object_name))
          myReqDet.point_cloud_object_details = object_list.graspable_objects[i].region.cloud
          obj_det_resp = object_details(myReqDet)
          #raw_input("Waiting for a keystroke")
          ist_object_list.objects.append(obj_det_resp.object)
    except rospy.ServiceException, e:
        print "GetObjectDetails Service call failed: %s"%e
        return False
    return ist_object_list


  def reset_point_cloud(self):
    print 'waiting for reset point cloud service...'
    rospy.wait_for_service('ist_reset_point_cloud')
    try:
        resetPointCloud = rospy.ServiceProxy('ist_reset_point_cloud',std_srvs.srv.Empty)
        myReq = std_srvs.srv.EmptyRequest()
        resetPointCloud(myReq)
    except rospy.ServiceException, e:
        print "Reset Point Cloud Service call failed: %s"%e
        return False
    
    return True
        

  def execution_steps(self,goal):
       
    # 0. Reset visualization point clouds  
    # publish the feedback
    self._feedback.state="Reseting visualization point clouds..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)
    
    # Service call
    reset_resp=self.reset_point_cloud()
    if reset_resp==False:
        #self._as.set_aborted(self._result)
        return False

    # 1. Segmentation

    # publish the feedback
    self._feedback.state="Executing tabletop segmentation..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)

    # Service call


    segmentation_resp=self.tabletop_segmentation(self._result.table)     

    if segmentation_resp==False:
        self._as.set_aborted(self._result)
        return False

    #self._first=False
    #self._result.table = segmentation_resp.table
    #self._result.clusters = segmentation_resp.clusters
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

    object_list = object_recognition_and_pose_estimation_resp.object_list

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
    object_recognition_and_pose_estimation_resp=self.object_details(object_list)
    if object_list==False:
        #self._as.set_aborted(self._result)
        return False
    #print_objects_info(object_list)  
     
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

