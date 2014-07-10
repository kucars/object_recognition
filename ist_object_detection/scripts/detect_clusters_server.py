#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_object_detection')
import rospy
roslib.load_manifest('ist_generate_grasps')
import actionlib
roslib.load_manifest('ist_tasks')
import perception_msgs.msg
import perception_msgs.srv
from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *
import ist_msgs.msg
import std_srvs.srv
from tabletop_object_detector.srv import *
import tf
import numpy as np
import math
import time
#from wsg_gripper.msg import *
import omnirob_controller_msgs
from omnirob_controller_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from orca_proxy.srv import *
from geometry_msgs.msg import Quaternion

GRIPPER_SIM_OPENING_THRESHOLD=0.01

def rotMatrixToQuaternion(my_rot_mat):
  if 1+my_rot_mat[0][0]+my_rot_mat[1][1]+my_rot_mat[2][2] < 0:
    return np.array([-1,-1,-1,-1])
  else:
    q_w = math.sqrt(0.5*(1+my_rot_mat[0][0]+my_rot_mat[1][1]+my_rot_mat[2][2]))
    q_x = 0.25*(my_rot_mat[2][1]-my_rot_mat[1][2])/q_w
    q_y = 0.25*(my_rot_mat[0][2]-my_rot_mat[2][0])/q_w
    q_z = 0.25*(my_rot_mat[1][0]-my_rot_mat[0][1])/q_w
    return np.array([q_x,q_y,q_z,q_w])

def numpyToMsg (hmatrix):
    pose=geometry_msgs.msg.Pose() 
    quaternion_array=tf.transformations.quaternion_from_matrix(hmatrix)
    pose.orientation=geometry_msgs.msg.Quaternion()
    pose.orientation.x=quaternion_array[0]
    pose.orientation.y=quaternion_array[1]
    pose.orientation.z=quaternion_array[2]
    pose.orientation.w=quaternion_array[3]
    pose.position=geometry_msgs.msg.Point()
    position_array=tf.transformations.translation_from_matrix(hmatrix)
    pose.position.x=position_array[0]
    pose.position.y=position_array[1]
    pose.position.z=position_array[2]
    return pose

def gripperGraspingSim():
    joint_states=rospy.wait_for_message('joint_states', sensor_msgs.msg.JointState)
    if joint_states.position[7]>=GRIPPER_SIM_OPENING_THRESHOLD:
           return True
    else:
           return False

def makePoint(position):
    mypoint = JointTrajectoryPoint()    
    mypoint.positions = position
    mypoint.velocities = [ 0, 0, 0, 0, 0, 0, 0 ]
    mypoint.accelerations = [ 0, 0, 0, 0, 0, 0, 0 ]
    return mypoint

def move_arm_home_position_traj():

    print 'Move arm to home position'
    mytrajectory = JointTrajectory()

    mytrajectory.points.append(makePoint([ 0, 0, 0, 0, 0, 0, 0 ]))
    str = "Publishing Trajectory to Kerze %s" % rospy.get_time()
    return move_arm(mytrajectory)

def query_yes_no(question, default=None):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is one of "yes" or "no".
    """
    valid = {"y":True, "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")

def somethingWentWrong(error_msg):
    print 'Something went wrong while ' + str(error_msg) + ' Fix the problem and repeat experiment'
    while not query_yes_no("Problem fixed? (yes)"):
        print 'fix problem...'
        if query_yes_no("Skip prob? (yes)"):
            break

def move_arm(trajectory):
    print "Waiting for left joint trajectory action server to start.";
    client = actionlib.SimpleActionClient('left_joint_trajectory_action', omnirob_controller_msgs.msg.JointTrajectoryAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print 'Done'

     # Creates a goal to send to the action server.
    goal = omnirob_controller_msgs.msg.JointTrajectoryGoal(trajectory=trajectory)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print '    Wait for result...'
    client.wait_for_result()
    if client.get_result().finish==-1:
           print 'Arm didnt move'
           return False
    elif client.get_result().finish==-2:
           print 'Robot is probabily dead'
           return False
    else:
        print 'Arm moved'
    return True

def collision_environment(table, action):
    print 'Waiting for collision environment service...'
    rospy.wait_for_service('new_object_collision')
    try:
        collision_srv = rospy.ServiceProxy('new_object_collision' , ist_grasp_generation_msgs.srv.AddObjectCollision)
        myReq = ist_grasp_generation_msgs.srv.AddObjectCollisionRequest()
        myReq.table=table
        #myReq.object_list=object_list
        myReq.action=action
        collision_srv(myReq)
    except rospy.ServiceException, e:
        print "Collision environment service call failed: %s"%e
        return False
    return True

class DetectClustersAction(object):
  # create messages that are used to publish feedback/result
  _feedback = perception_msgs.msg.DetectClustersFeedback()
  _result   = perception_msgs.msg.DetectClustersResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, perception_msgs.msg.DetectClustersAction, execute_cb=self.execute_cb)
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
        self._result.clusters_list = object_list
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
        self._as.set_aborted(self._result)
        return False

    
    # 1. Segmentation

    # publish the feedback
    self._feedback.state="Executing tabletop segmentation..." 
    self._feedback.progress=33.0
    self._as.publish_feedback(self._feedback)
    
    move_arm_home_position_traj()
    self.set_object_position_service(goal)
    # Service call
    segmentation_resp=self.tabletop_segmentation()
    if segmentation_resp==False:
        self._as.set_aborted(self._result)
        return False

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    

    
    # 2. Normals computation and subsampling

    # publish the feedback
    self._feedback.state="Executing normal computation and subsampling." 
    self._feedback.progress=67.0
    self._as.publish_feedback(self._feedback)
   
    # Service call
    normals_and_subsampling_resp=self.normals_and_subsampling(segmentation_resp)
    if normals_and_subsampling_resp==False:
        self._as.set_aborted(self._result)
        return False
    #my_object_list=[]
    collision_environment(segmentation_resp.table, 1)
    for i in range(0,len(normals_and_subsampling_resp)):
      currentPointCloud = normals_and_subsampling_resp[i]
      grasping_labels = self.planning_and_grasping(currentPointCloud,goal)    
      self._result.grasp_label_pose_1 = grasping_labels[0]
      self._result.grasp_label_pose_2 = grasping_labels[1]
      self._result.grasp_pose_1 = grasping_labels[2]
      self._result.grasp_pose_2 = grasping_labels[3]
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
   
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
        
    # publish the feedback
    self._feedback.state="Done." 
    self._feedback.progress=100.0
    self._as.publish_feedback(self._feedback)
    
    return normals_and_subsampling_resp
    
  def reset_point_cloud(self):
    print 'waiting for reset point cloud service...'
    rospy.wait_for_service('ist_reset_point_cloud_not_comp')
    try:
        resetPointCloud = rospy.ServiceProxy('ist_reset_point_cloud_not_comp',std_srvs.srv.Empty)
        myReq = std_srvs.srv.EmptyRequest()
        resetPointCloud(myReq)
    except rospy.ServiceException, e:
        print "Reset Point Cloud Service call failed: %s"%e
        return False
    
    return True

  def normals_and_subsampling(self,segmentation_resp):    
    print 'waiting for cluster subsampling and normal computation service...'
    rospy.wait_for_service('ist_compute_object_normals')
    try:
        point_refinement = rospy.ServiceProxy('ist_compute_object_normals' , perception_msgs.srv.GetSubSampledNormalsPointCloud)
        myReq = perception_msgs.srv.GetSubSampledNormalsPointCloudRequest()
        object_list = []
        #self.request_base_link_to_table_tf()
        #object_list = ist_msgs.msg.ObjectList()
        for i in range(0,len(segmentation_resp.clusters)):
            myReq.point_cloud = segmentation_resp.clusters[i]
            #myReq.object_name = std_msgs.msg.String(str(object_name))
            myReq.table = segmentation_resp.table
            complete_shape = point_refinement(myReq)
            subsampledPC = ist_msgs.msg.SubsampledPointCloud()
            subsampledPC.object_point_cloud = complete_shape.point_cloud
            #for j in range(0,len(complete_shape.point_cloud.channels)):
            #print complete_shape.point_cloud.channels[j].name
            object_list.append(subsampledPC)
    except rospy.ServiceException, e:
        print "GetSubSampledNormalsPointCloud Service call failed: %s"%e
        return False
        
    return object_list

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

  def set_object_position_service(self, goal):
    print 'waiting for orca set object position service...'
    rospy.wait_for_service('orca_set_object_position')
    set_position = rospy.ServiceProxy('orca_set_object_position',SetObjectPosition)
    try:
      resp = set_position(goal.object_name,goal.position, goal.axes)#place object
    except rospy.ServiceException, e:
      print "set position service call failed: %s"%e

  def planning_and_grasping(self,currentPointCloud,goal):
    orientation_one_result = []
    orientation_two_result = []
    quaternion_one = []
    quaternion_two = []
    for pointCounter in range(0,len(currentPointCloud.object_point_cloud.points)):
      currentPoint = currentPointCloud.object_point_cloud.points[pointCounter]
      currentNormal_X = currentPointCloud.object_point_cloud.channels[1].values[pointCounter]
      currentNormal_Y = currentPointCloud.object_point_cloud.channels[2].values[pointCounter]
      currentNormal_Z = currentPointCloud.object_point_cloud.channels[3].values[pointCounter]
      current_p_x = currentPointCloud.object_point_cloud.points[pointCounter].x
      current_p_y = currentPointCloud.object_point_cloud.points[pointCounter].y
      current_p_z = currentPointCloud.object_point_cloud.points[pointCounter].z
      my_Z = np.array([0,0,1])
      my_Y = np.array([0,1,0])
      my_X = np.array([1,0,0])
      my_N = 1*np.array([currentNormal_X,currentNormal_Y,currentNormal_Z])
      #my_Z_cross_my_N = np.cross(my_Z,my_N)
      projected_Z = np.cross(my_N,np.cross(my_Z,my_N/np.linalg.norm(my_N)))/np.linalg.norm(my_N)
      projected_Y = np.cross(my_N,np.cross(my_Y,my_N/np.linalg.norm(my_N)))/np.linalg.norm(my_N)
      projected_X = np.cross(my_N,np.cross(my_X,my_N/np.linalg.norm(my_N)))/np.linalg.norm(my_N)
      norm_proj_Z = np.linalg.norm(projected_Z)
      norm_proj_Y = np.linalg.norm(projected_Y)
      norm_proj_X = np.linalg.norm(projected_X)
      if norm_proj_Y < norm_proj_X and norm_proj_Y < norm_proj_Z:
        projected_Y = -1*projected_X
        #projected_Z = 1*projected_Z
      if norm_proj_Z < norm_proj_X and norm_proj_Z < norm_proj_Y:
        projected_Z = -1*projected_X
      #print projected_Z,projected_Y
      #print type(my_N)
      #The Y is oriented parallel to the fingers and pointing away from the gripper.
      #Z is perpendicular to the gripper piece (the flat piece that says schunk) and 
      #X is on the direction of the longest side of the gripper piece.
      z_pose_1 = 1*projected_Y/np.linalg.norm(projected_Y)
      x_pose_1 = 1*projected_Z/np.linalg.norm(projected_Z)
      y_pose_1 = my_N/np.linalg.norm(my_N)
      z_pose_2 = 1*projected_Z/np.linalg.norm(projected_Z)
      x_pose_2 = 1*projected_Y/np.linalg.norm(projected_Y)
      y_pose_2 = my_N/np.linalg.norm(my_N)
      #print x_pose_1,y_pose_1,my_N,z_pose_1,x_pose_2,y_pose_2,z_pose_2
      rot_mat_pose_1 = np.array([[np.dot(my_X,x_pose_1),np.dot(my_Y,x_pose_1),np.dot(my_Z,x_pose_1)],[np.dot(my_X,y_pose_1),np.dot(my_Y,y_pose_1),np.dot(my_Z,y_pose_1)],[np.dot(my_X,z_pose_1),np.dot(my_Y,z_pose_1),np.dot(my_Z,z_pose_1)]])
      rot_mat_pose_2 = np.array([[np.dot(my_X,x_pose_2),np.dot(my_Y,x_pose_2),np.dot(my_Z,x_pose_2)],[np.dot(my_X,y_pose_2),np.dot(my_Y,y_pose_2),np.dot(my_Z,y_pose_2)],[np.dot(my_X,z_pose_2),np.dot(my_Y,z_pose_2),np.dot(my_Z,z_pose_2)]])
      #print rot_mat_pose_1
      #print rot_mat_pose_2
      quaternion_pose_1 = rotMatrixToQuaternion(rot_mat_pose_1)
      point_location = np.array([current_p_x,current_p_y,current_p_z])
      if not np.array_equal(quaternion_pose_1, np.array([-1,-1,-1,-1])):
        planner_response = self.plan_trajectory(point_location,quaternion_pose_1)
        #raw_input("Press Enter to continue...")
        #time.sleep(0.2)
        if planner_response.success == True:
          print 'Solution found pose 1'
          #self.openGripper()
          self.set_object_position_service(goal)
          curr_success = self.graspingAction(planner_response.trajectory)
          if curr_success:
            orientation_one_result.append(1)
          else:
            orientation_one_result.append(0)
          quaternion_one.append(Quaternion(x=quaternion_pose_1[0],y=quaternion_pose_1[1],z=quaternion_pose_1[2],w=quaternion_pose_1[3]))
        else:
          orientation_one_result.append(2)
          quaternion_one.append(Quaternion(x=-1.0,y=-1.0,z=-1.0,w=-1.0))
          print 'Solution not found pose 1'
      else:
        z_pose_1 = -1*projected_Y/np.linalg.norm(projected_Y)
        x_pose_1 = -1*projected_Z/np.linalg.norm(projected_Z)
        y_pose_1 = my_N/np.linalg.norm(my_N)
        rot_mat_pose_1 = np.array([[np.dot(my_X,x_pose_1),np.dot(my_Y,x_pose_1),np.dot(my_Z,x_pose_1)],[np.dot(my_X,y_pose_1),np.dot(my_Y,y_pose_1),np.dot(my_Z,y_pose_1)],[np.dot(my_X,z_pose_1),np.dot(my_Y,z_pose_1),np.dot(my_Z,z_pose_1)]])
        quaternion_pose_1 = rotMatrixToQuaternion(rot_mat_pose_1)
        if not np.array_equal(quaternion_pose_1, np.array([-1,-1,-1,-1])):
          planner_response = self.plan_trajectory(point_location,quaternion_pose_1)
          #time.sleep(0.2)
          #raw_input("Press Enter to continue...")
          if planner_response.success == True:
            print 'Solution found pose 1 option 2'
            self.set_object_position_service(goal)
            curr_success = self.graspingAction(planner_response.trajectory)
            if curr_success:
              orientation_one_result.append(1)
            else:
              orientation_one_result.append(0)
            quaternion_one.append(Quaternion(x=quaternion_pose_1[0],y=quaternion_pose_1[1],z=quaternion_pose_1[2],w=quaternion_pose_1[3]))
          else:
            orientation_one_result.append(2)
            quaternion_one.append(Quaternion(x=-1.0,y=-1.0,z=-1.0,w=-1.0))
            print 'Solution not found pose 1 option 2'
      #move_arm_home_position_traj()
      quaternion_pose_2 = rotMatrixToQuaternion(rot_mat_pose_2)
      if not np.array_equal(quaternion_pose_2, np.array([-1,-1,-1,-1])):
        planner_response = self.plan_trajectory(point_location,quaternion_pose_2)
        #time.sleep(0.2)
        #raw_input("Press Enter to continue...")
        if planner_response.success == True:
          print 'Solution found pose 2'
          self.set_object_position_service(goal)
          curr_success = self.graspingAction(planner_response.trajectory)
          if curr_success:
            orientation_two_result.append(1)
          else:
            orientation_two_result.append(0)
          quaternion_two.append(Quaternion(x=quaternion_pose_2[0],y=quaternion_pose_2[1],z=quaternion_pose_2[2],w=quaternion_pose_2[3]))
        else:
          orientation_two_result.append(2)
          quaternion_two.append(Quaternion(x=-1.0,y=-1.0,z=-1.0,w=-1.0))
          print 'Solution not found pose 2'
      else:
        z_pose_2 = -1*projected_Z/np.linalg.norm(projected_Z)
        x_pose_2 = -1*projected_Y/np.linalg.norm(projected_Y)
        y_pose_2 = my_N/np.linalg.norm(my_N)
        rot_mat_pose_2 = np.array([[np.dot(my_X,x_pose_2),np.dot(my_Y,x_pose_2),np.dot(my_Z,x_pose_2)],[np.dot(my_X,y_pose_2),np.dot(my_Y,y_pose_2),np.dot(my_Z,y_pose_2)],[np.dot(my_X,z_pose_2),np.dot(my_Y,z_pose_2),np.dot(my_Z,z_pose_2)]])
        quaternion_pose_2 = rotMatrixToQuaternion(rot_mat_pose_2)
        if not np.array_equal(quaternion_pose_2, np.array([-1,-1,-1,-1])):
          planner_response = self.plan_trajectory(point_location,quaternion_pose_2)
          #time.sleep(0.2)
          #raw_input("Press Enter to continue...")
          if planner_response.success == True:
            print 'Solution found pose 2 option 2'
            self.set_object_position_service(goal)
            curr_success = self.graspingAction(planner_response.trajectory)
            if curr_success:
              orientation_two_result.append(1)
            else:
              orientation_two_result.append(0)
            quaternion_two.append(Quaternion(x=quaternion_pose_2[0],y=quaternion_pose_2[1],z=quaternion_pose_2[2],w=quaternion_pose_2[3]))
          else:
            orientation_two_result.append(2)
            quaternion_two.append(Quaternion(x=-1.0,y=-1.0,z=-1.0,w=-1.0))
            print 'Solution not found pose 2 option 2'
      #print quaternion_pose_1,quaternion_pose_2
      #print orientation_one_result
    #resultList=[]
    #resultList.append(orientation_one_result)
    #resultList.append(orientation_two_result)
    return [orientation_one_result,orientation_two_result,quaternion_one,quaternion_two]#resultList

  def plan_trajectory(self,point_location,quaternion_orientation):
    #receives a quaternion array and a translation array
    print 'waiting for motion planning service...'
    rospy.wait_for_service('motion_planning')
    planner_response=MotionPlanResponse()
    
    grasping_point_hmatrix=np.mat(tf.transformations.quaternion_matrix(quaternion_orientation)+tf.transformations.translation_matrix(point_location)-tf.transformations.identity_matrix())
    
    
    pose_stamped=geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id='base_link'
    pose_stamped.header.stamp=rospy.Time.now()
    pose_stamped.pose=numpyToMsg(grasping_point_hmatrix)
    
    try:      
      plan_trajectory_srv = rospy.ServiceProxy('motion_planning',MotionPlan)
      myReq = MotionPlanRequest()
      myReq.mode=1

      myReq.pose=pose_stamped
      planner_response=plan_trajectory_srv(myReq)
    except rospy.ServiceException, e:
      print "Motion planning service call failed: %s"%e
    return planner_response

  def openGripper(self):
    client = actionlib.SimpleActionClient('/move_gripper_action', omnirob_controller_msgs.msg.MoveGripperAction)

    print "Waiting for gripper command action server to start.";
    client.wait_for_server()
    print "Gripper command action server started, sending goal.";
    # Waits until the action server has started up and started
    # listening for goals.

     # Creates a goal to send to the action server.
    command_gripper_goal = omnirob_controller_msgs.msg.MoveGripperGoal()
    command_gripper_goal.open_gripper = True;

    # Sends the goal to the action server.
    client.send_goal(command_gripper_goal)

    # Waits for the server to finish performing the action.
    print '    Wait for result...'
    client.wait_for_result()
    print 'Success code:', client.get_result().finish; 
    if client.get_result().finish:
        print 'yes!'
        return True
    else:
        return False


  def closeGripper(self):
    client = actionlib.SimpleActionClient('/move_gripper_action', omnirob_controller_msgs.msg.MoveGripperAction)

    print "Waiting for gripper command action server to start.";
    client.wait_for_server()
    print "Gripper command action server started, sending goal.";
    # Waits until the action server has started up and started
    # listening for goals.

     # Creates a goal to send to the action server.
    command_gripper_goal = omnirob_controller_msgs.msg.MoveGripperGoal()
    command_gripper_goal.open_gripper = False;
    
    # Sends the goal to the action server.
    client.send_goal(command_gripper_goal)

    # Waits for the server to finish performing the action.
    print '    Wait for result...'
    client.wait_for_result()
    print 'Success code:', client.get_result().finish; 
    if client.get_result().finish == 2:
        print 'Grasping!!!'
        return 2
    elif client.get_result().finish==1:
        print 'Not grasping but closed'
        return 1
    else:
        return -1
  
  def graspingAction(self,trajectory):
    trial_attempt=0
    good_grasp_trial=False
            
    while True:
      success=False
      trial_attempt=trial_attempt+1
      print 'trial attempt: ' + str(trial_attempt)
        
      # Open gripper when in home position...
      #####self.closeGripper()
      while not self.openGripper():
        somethingWentWrong('opening the gripper')
        
      # Move to grasp position
      #while not move_arm(trajectory):
      if not move_arm(trajectory):
        #somethingWentWrong('moving the arm')
        success=False
        move_arm_home_position_traj()
        return success

      # Close gripper... Grasp object                
      while self.closeGripper()==-1:
        somethingWentWrong('closing the gripper')

      rospy.sleep(rospy.Duration(2))
        
      # Check if grasping
      if gripperGraspingSim():
        success=True
        print 'is grasping!'
      else:
        print 'is not grasping...'
      # If it's not grasping open gripper
      if not success:
        # Open gripper
        if not self.openGripper():
          somethingWentWrong('opening the gripper')
      move_arm_home_position_traj()
      return success


if __name__ == '__main__':
  rospy.init_node('detect_clusters_server')
  DetectClustersAction(rospy.get_name())
  rospy.spin()

