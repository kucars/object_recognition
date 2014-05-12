#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_object_detection')
import rospy

import actionlib

import perception_msgs.msg
import perception_msgs.srv
import ist_msgs.msg
import std_srvs.srv
import problog_msgs.srv
from tabletop_object_detector.srv import *
import visualization_msgs.msg
import copy
use_marion=True
topic = 'object_categories'
publisher = rospy.Publisher(topic, visualization_msgs.msg.MarkerArray)
markerArray = visualization_msgs.msg.MarkerArray()


# def delete_markers():
#   for marker_index in range(0,len(markerArray.markers)):
#     marker = visualization_msgs.msg.Marker()
#     # ... here I get the data I want to plot into a vector called trans
#     #marker.header.frame_id = frame_id
#     marker.header.frame_id = markerArray.markers[marker_index].header.frame_id
#     #marker.header.stamp = rospy.rostime.get_time()
#     marker.type = markerArray.markers[marker_index].type
#     #marker.lifetime = rospy.rostime.Duration
#     marker.text = markerArray.markers[marker_index].text
#     marker.ns = markerArray.markers[marker_index].ns
#     marker.action = marker.DELETE
#   publisher.publish(markerArray)
#   for marker_index in range(0,len(markerArray.markers)):
#     markerArray.markers.pop()
     
def add_marker(text,frame_id,text_pose):
   marker = visualization_msgs.msg.Marker()
   # ... here I get the data I want to plot into a vector called trans
   #marker.header.frame_id = frame_id
   marker.header.frame_id = "/base_link"
   #marker.header.stamp = rospy.rostime.get_time()
   marker.type = marker.TEXT_VIEW_FACING
   marker.lifetime = rospy.rostime.Duration(secs=1, nsecs=0)
   marker.text = text
   marker.ns = text+str(len(markerArray.markers))
   marker.action = marker.ADD
   marker.scale.x = 0.1
   marker.scale.y = 0.1
   marker.scale.z = 0.1
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.pose=text_pose

   # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
#    if(count > MARKERS_MAX):
#      markerArray.markers.pop(0)
#    else:
#      count += 1
   
   markerArray.markers.append(marker)
   




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
        self._result.object_list = object_list
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
    else:
        self._result.object_list=ist_msgs.msg.ObjectList()
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
        self._result.table=resp.table
        return resp
    except rospy.ServiceException, e:
        print "TabletopSegmentation Service call failed: %s"%e
        return False
        
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
    self._feedback.progress=20.0
    self._as.publish_feedback(self._feedback)
    
    # Service call
    segmentation_resp=self.tabletop_segmentation()
    if segmentation_resp==False:
        #self._as.set_aborted(self._result)
        return False

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    

    
    # 2. Shape completion

    # publish the feedback
    self._feedback.state="Executing shape completion." 
    self._feedback.progress=40.0
    self._as.publish_feedback(self._feedback)
   
    # Service call
    shape_completion_resp=self.shape_completion(segmentation_resp)
    if shape_completion_resp==False:
        #self._as.set_aborted(self._result)
        return False
        
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        #self._as.set_preempted()
        return False
        
     # 3. Object details

    # publish the feedback
    self._feedback.state="Computing object details." 
    self._feedback.progress=60.0
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
    self._feedback.progress=80.0
    self._as.publish_feedback(self._feedback)
    
    # 4. Object classification
    if use_marion:
        rospy.set_param('/marion_activated', True)
        # publish the feedback
        self._feedback.state="Compute objects' categories..." 
        self._feedback.progress=100
        self._as.publish_feedback(self._feedback)
    else:
        rospy.set_param('/marion_activated', False)
    
    # Service call
    object_category_response=self.compute_object_category_prior(object_list)
    if object_category_response==False:
        #self._as.set_aborted(self._result)
        return False
    if use_marion:
        print "Objects' categories info:"
        object_list=object_category_response
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    
    # sort according to category
#     delete_markers()
    markerArray.markers=[]
    for object_index in range(0,len(object_list.objects)):
        object_list.objects[object_index].data.category_hypotheses=self.getSortedCategoryList(object_list.objects[object_index].data.category_hypotheses)
        object_frame_id=str(object_list.objects[object_index].collision_name)
        #print object_frame_id

        object_category=str(object_list.objects[object_index].data.category_hypotheses[0].name)
        #print object_category

        text_pose=copy.deepcopy(object_list.objects[object_index].state.graspable_object.potential_models[0].pose.pose)
        text_pose.position.z=text_pose.position.z+object_list.objects[object_index].data.type.size.values.z+0.02

        add_marker(object_category,object_frame_id,text_pose)
    print_objects_info(object_category_response)  

    

    return object_list
    
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

  def shape_completion(self,segmentation_resp):    
    print 'waiting for shape completion service...'
    rospy.wait_for_service('ist_point_cloud_refinement')
    try:
        point_refinement = rospy.ServiceProxy('ist_point_cloud_refinement' , perception_msgs.srv.GetRefinedPointCloud)
        myReq = perception_msgs.srv.GetRefinedPointCloudRequest()
        #object_list = []
        object_list = ist_msgs.msg.ObjectList()
        for i in range(0,len(segmentation_resp.clusters)):
            myReq.point_cloud = segmentation_resp.clusters[i]
            #myReq.object_name = std_msgs.msg.String(str(object_name))
            myReq.table = segmentation_resp.table
            complete_shape = point_refinement(myReq)
            object_list.objects.append(complete_shape)
    except rospy.ServiceException, e:
        print "GetRefinedPointCloud Service call failed: %s"%e
        return False
        
    return object_list


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

  # Object category prior service
  def compute_object_category_prior(self, object_list):
    print 'waiting for compute prior service...'
    rospy.wait_for_service('compute_prior')
    try:
        compute_prior_srv = rospy.ServiceProxy('compute_prior' , problog_msgs.srv.ObjectInference)
        myReq = problog_msgs.srv.ObjectInferenceRequest()
        myReq.object_list=object_list
        resp = compute_prior_srv(myReq)
        object_list_resp=resp.object_list
    except rospy.ServiceException, e:
        print "Compute prior Service call failed: %s"%e
        return False
    return object_list_resp

  def getSortedCategoryList(self, category_hypotheses):#,object_id):
    probabilities=[]
    for category_index in range(0,len(category_hypotheses)):
      probabilities.append(category_hypotheses[category_index].likelihood)

    sorted_indexes = [i[0] for i in sorted(enumerate(probabilities), key=lambda x:x[1], reverse=True)]
    sorted_category_list=[]
    for sorted_indexes_index in range(0,len(sorted_indexes)):
      #print 'before -> category:'+category_hypotheses[sorted_indexes[sorted_indexes_index]].name+'prob:' + str(probabilities[sorted_indexes[sorted_indexes_index]]) + ' '
      sorted_category_list.append(category_hypotheses[sorted_indexes[sorted_indexes_index]])
    category_hypotheses=sorted_category_list
    #for category_index in range(0,len(category_hypotheses)):
      #print 'after -> category:'+category_hypotheses[category_index].name+'prob:' + str(category_hypotheses[category_index].likelihood) + ' '
  
    return category_hypotheses


      
if __name__ == '__main__':
  rospy.init_node('detect_objects_server')
  DetectObjectsAction(rospy.get_name())

  while not rospy.is_shutdown():
    publisher.publish(markerArray)
    rospy.sleep(0.1)
  
