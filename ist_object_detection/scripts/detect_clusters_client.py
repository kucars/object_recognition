#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_tasks')
import rospy
import actionlib
from perception_msgs.msg import *
from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *
import numpy
from orca_proxy.srv import *
from geometry_msgs.msg import Point
import math
from array import array
import time

class ObjectTest:
    def __init__(self, name, sim_id, orientations, orientation_base, offset, category):
        self.name = name
        self.sim_id = sim_id
        self.orientations = orientations
        self.orientation_base = orientation_base
        self.offset = offset
        self.category = category

def perception_client():
	# Region bounding the plannar surface and the objects of interest 
	table_region=perception_msgs.msg.TableRegion() 
	table_region=perception_msgs.msg.TableRegion()

	#UPPER SHELF
	table_region.x_filter_max=1.4
	table_region.x_filter_min=0.7
	table_region.y_filter_max=0.6
	table_region.y_filter_min=-0.6
	table_region.z_filter_max=1.5
	table_region.z_filter_min=0.9
	objectList = [              ObjectTest('griddle','freeform_griddle120',                     8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0.2,  0.0,  0   ]), 'can'),       # usar
                                #ObjectTest('small_griddle','freeform_frying_pan138',          8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    -0.01   ]), 'can'),       # usar
                                ObjectTest('old_frying_pan','freeform_old_frying_pan139',       8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0,    0,    0   ]), 'can'),      # usar
                                #ObjectTest('long_pan','freeform_pan_long113',                   8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0,    0,    0.03]), 'can')]
                                #ObjectTest('medium_bowl','freeform_bowl2128',                   1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can'),
                                #ObjectTest('big_bowl','freeform_bowl3129',                      1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can'),
                                ObjectTest('ikea_mug','freeform_mug_ikea106',                   8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0.15,    -0.15,    0   ]), 'can'),     # usar
                                ObjectTest('short_mug','freeform_mug_short107',                 8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    -0.05,    0   ]), 'can'),   # usar
                                ObjectTest('large_mug','freeform_mug_large108',                 8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    -0.05,    0   ]), 'can'), # usar
                                #ObjectTest('cocktail_glass','freeform_cocktail_glass131',       1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can')]       # usar
                                ObjectTest('wine_glass','freeform_wine_glass132',     1, numpy.matrix([[1, 0, 0],[0, 0, -1],[0, 1, 0]]), numpy.array([0,    0.3,    0   ]), 'can'),             # usar
                                ObjectTest('champagne_glass','freeform_glass_champagne104',     1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can'),      # usar
                                ObjectTest('cognac_glass','freeform_glass_cognac103',           1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can'),
                                ObjectTest('beer_bottle','freeform_beer_bottle125',     1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0   ]), 'can'),              # usar
                                #ObjectTest('lying_beer_bottle','freeform_beer_bottle125',     8, numpy.matrix([[1, 0, 0],[0, 0, -1],[0, 1, 0]]), numpy.array([0,    0,    0.05   ]), 'can'),              # usar
                                ObjectTest('booze_bottle','freeform_booze_bottle118',           1, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0,    0,    0.17]), 'can'),      # usar
                                #ObjectTest('lying_booze_bottle','freeform_booze_bottle118',           8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0,    0,    0.1]), 'can'),      # usar
                                #ObjectTest('wine_bottle','freeform_wine_bottle127',             1, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0 ,   0,    0.17]), 'can'),      # usar
                                ObjectTest('champagne_bottle','freeform_champagne_bottle126',   1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0 ,   0,    0   ]), 'can'),
                                ObjectTest('rounded_can','freeform_can_rounded110',             4, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0.3 , 0,    0   ]), 'can'),      # usar
                                ObjectTest('squared_can','freeform_can_square109',              4, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0.3 , 0,    0   ]), 'can'),      # usar
                                ObjectTest('ellipsoid_can','freeform_can_ellipsoid111',         4, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0.3 , 0,    0   ]), 'can'),      # usar
                                #ObjectTest('long_pot','freeform_pot_long125',                   8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0 ,   0,    0   ]), 'can'),
                                #ObjectTest('coffeemaker','freeform_coffemaker115',              8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0.3 , 0,    0.1 ]), 'can'),
                                #ObjectTest('short_pot','freeform_pot_short112',                 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0 ,   0,    0   ]), 'can'),   
                                #ObjectTest('martini_glass','freeform_martini_glass130',         1, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]), numpy.array([0 ,   0.3,  0]),       'can'),
                                ObjectTest('wide_hammer','freeform_wide_hammer134',             8, numpy.matrix([[0, 0, 1],[0, 1, 0],[1, 0, 0]]) , numpy.array([0 ,   0,    0.155]), 'can'),
                                ObjectTest('claw_hammer','freeform_claw_hammer123',             8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, -1, 0]]), numpy.array([0 ,   0,  0]),       'can'),
                                ObjectTest('hunting_knife','freeform_hunting_knife117',         8, numpy.matrix([[0, 1, 0],[1, 0, 0],[0, 0, 1]]) , numpy.array([0 ,   0,    0.03]),  'can'),
                                ObjectTest('butcher_knife','freeform_butcher_knife136',         8, numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) , numpy.array([0.05 ,   0.06,  0]),       'can'),
                                ObjectTest('small_screwdriver','freeform_screwdriver_small142', 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]) , numpy.array([0 ,   0,  0]),       'can'),
                                ObjectTest('big_screwdriver','freeform_screwdriver_big_two143', 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]) , numpy.array([0 , 0.1,  0]),       'can')]
    #objectList = [#ObjectTest('wide_hammer','freeform_wide_hammer135',                           8, numpy.matrix([[0, 0, 1],[0, 1, 0],[1, 0, 0]]), numpy.array([0 ,   0,    0.155]), 'can')]
                        #ObjectTest('hunting_knife','freeform_hunting_knife118',         8, numpy.matrix([[0, 1, 0],[1, 0, 0],[0, 0, 1]]), numpy.array([0 ,   0,    0.03]),  'can')]
                        #ObjectTest('big_screwdriver','freeform_screwdriver_big_two144', 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0 ,   0,  0]),       'can')]
                        #ObjectTest('small_screwdriver','freeform_screwdriver_small143', 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, 1, 0]]), numpy.array([0 ,   0,  0]),       'can')]                
                        #ObjectTest('claw_hammer','freeform_claw_hammer124', 8, numpy.matrix([[1, 0, 0],[0, 0, 1],[0, -1, 0]]), numpy.array([0 ,   0,  0]),       'can')]    

	# Creates the SimpleActionClient, passing the type of the action (DetectObjectsAction) to the constructor.
	client = actionlib.SimpleActionClient('detect_clusters_server', perception_msgs.msg.DetectClustersAction)

	# Waits until the action server has started up and started listening for goals.
	client.wait_for_server()

	print 'Object list length: ' + str(len(objectList))
	for elemCounter in range(len(objectList)):
		print 'waiting for orca set object position service...'
		rospy.wait_for_service('orca_set_object_position')
		try:
			set_position = rospy.ServiceProxy('orca_set_object_position',SetObjectPosition)
			position_array = numpy.mat(numpy.array([-4.5, 4.9, 1.0]))+numpy.mat(objectList[elemCounter].offset)
			print position_array
			position = Point(x=position_array[0,0],y=position_array[0,1],z=position_array[0,2])
			floorPosition = Point(x=-5.5,y=3.8,z=1.0)#-5.5,4.8,1.0
			axesFloor = numpy.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
			for orient in range(0,objectList[elemCounter].orientations):
				f = open(objectList[elemCounter].name+str(orient), "w")
				print 'waiting for orca set object position service...'
				rospy.wait_for_service('orca_set_object_position')
				try:
					if orient == 0 and elemCounter > 0:
						resp = set_position(objectList[elemCounter-1].sim_id,floorPosition, array('d',(numpy.resize(axesFloor,(1,9))).flat))#place object
				except rospy.ServiceException, e:
					print "set position service call failed: %s"%e
				a=orient/float(objectList[elemCounter].orientations)*2.0*math.pi;
				print 'Current angle: ' + str(a)
			        #-sin(a), cos(a), 0, -cos(a), -sin(a), 0.0, 0.0, 0.0, 1.0
				axes = numpy.matrix([[-math.sin(a), math.cos(a), 0.0], [-math.cos(a), -math.sin(a), 0.0],[ 0.0, 0.0, 1.0]])*objectList[elemCounter].orientation_base
				axes_vector = array('d',(numpy.resize(axes,(1,9))).flat)
				#resp = set_position(objectList[elemCounter].sim_id,position, )#rotate object
                                # Creates a goal to send to the action server.
				goal = perception_msgs.msg.DetectClustersGoal()
				goal.table_region = table_region
				goal.object_name = objectList[elemCounter].sim_id
				goal.position = position
				goal.axes = axes_vector
				# Sends the goal to the action server.
				resp = set_position(objectList[elemCounter].sim_id,position, axes_vector)#rotate object
				#time.sleep(1)
				client.send_goal(goal)#, self.perception_done_cb, self.perception_active_cb, self.perception_feedback_cb)
                                #client.cancel_goal()
				client.get_state()

				# Waits for the server to finish performing the action.
				client.wait_for_result()
				if len(client.get_result().clusters_list) == 0:
					print 'Detected 0 objects.\n'
				#print "Result:", ', '+ str(client.get_result())
				f.write(str(client.get_result()))
				f.close()
                                #return False
				###print 'OBJECT LIST SIZEEEEEEEEEEEEEEEEEE:', str(len(client.get_result().clusters_list))
				###print 'Current orientation: ' + str(orient)
		except rospy.ServiceException, e:
			print "set position service call failed: %s"%e
	# Prints out the result of executing the action
	###return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('detect_object_clusters_client')
        perception_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

