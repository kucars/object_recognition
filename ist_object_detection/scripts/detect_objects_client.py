#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_object_detection')
import rospy
import actionlib
from perception_msgs.msg import *
from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *
import numpy
from geometry_msgs.msg import Point
import math
from array import array
import time

def perception_client(table_region):
	# Creates the SimpleActionClient, passing the type of the action (DetectObjectsAction) to the constructor.
	client = actionlib.SimpleActionClient('detect_objects_server', perception_msgs.msg.DetectObjectsAction)
	goal = perception_msgs.msg.DetectObjectsGoal();
	goal.table_region = table_region
	# Waits until the action server has started up and started listening for goals.
	print 'Waiting for server'
	client.wait_for_server()
	client.send_goal(goal)
	client.get_state()
	# Waits for the server to finish performing the action.
	print 'Waiting for result'
	client.wait_for_result()
	object_list = client.get_result()
	return object_list


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
	table_region=perception_msgs.msg.TableRegion()
	#UPPER SHELF
	table_region.x_filter_max=1.0
	table_region.x_filter_min=-1.0
	table_region.y_filter_max=1
	table_region.y_filter_min=-1
	table_region.z_filter_max=3.0
	table_region.z_filter_min=0.1

        rospy.init_node('detect_object_client')
	print 'Node initialized'
        perception_client(table_region)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

