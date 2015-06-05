#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_object_detection')
import rospy
import perception_msgs.msg
import perception_msgs.srv
import ist_msgs.msg
import std_srvs.srv
from tabletop_object_segmentation_online.srv import *

def tabletop_segmentation():
  print 'waiting for tabletop segmentation service...'
  rospy.wait_for_service('tabletop_segmentation')
  try:
    table_top = rospy.ServiceProxy('tabletop_segmentation' , TabletopSegmentation)
    resp = table_top()
    if len(resp.clusters) == 0:
      print 'No clusters found'
      return False
    else:
      #print resp.table
      return resp
  except rospy.ServiceException, e:
      print "TabletopSegmentation Service call failed: %s"%e
      return False

def reset_point_cloud():
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

def shape_completion(segmentation_resp):    
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

if __name__ == '__main__':
  #rospy.init_node('tabletoptest')
  table_region=perception_msgs.msg.TableRegion() 
  table_region=perception_msgs.msg.TableRegion()
  #UPPER SHELF
  table_region.x_filter_max=1.0
  table_region.x_filter_min=-1.0
  table_region.y_filter_max=1
  table_region.y_filter_min=-1
  table_region.z_filter_max=3.0
  table_region.z_filter_min=0.1
  rospy.set_param("/tabletop_segmentation/x_filter_max",  table_region.x_filter_max)
  rospy.set_param("/tabletop_segmentation/x_filter_min",  table_region.x_filter_min)
  rospy.set_param("/tabletop_segmentation/y_filter_max",  table_region.y_filter_max)
  rospy.set_param("/tabletop_segmentation/y_filter_min",  table_region.y_filter_min)
  rospy.set_param("/tabletop_segmentation/z_filter_max",  table_region.z_filter_max)
  rospy.set_param("/tabletop_segmentation/z_filter_min",  table_region.z_filter_min)
  reset_point_cloud()
  segmentation_response = tabletop_segmentation()
  object_list = shape_completion(segmentation_response)
