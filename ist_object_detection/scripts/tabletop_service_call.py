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
  tabletop_segmentation()
