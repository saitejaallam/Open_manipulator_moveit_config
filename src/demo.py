#! /usr/bin/env python

#Author: Sai Teja

import rospy
import actionlib
import copy
import random
from std_msgs.msg import Header
import moveit_commander
from moveit_python import PlanningSceneInterface,
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Pose,PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from moveit_msgs.msg import Grasp,GripperTranslation,PickupAction,MoveItErrorCodes,PickupGoal, PickupResult
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation, ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal,GenerateGraspsResult
from math import pi,radians, sqrt
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

def plan_motion():
  ## First initialize moveit_commander and rospy.
  #moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('simple_pick_place',
                  #anonymous=True)
  robot = RobotCommander()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group refers to the joints of
  ## arm. This interface can be used to plan and execute motions on arm and gripper.
  arm_group = moveit_commander.MoveGroupCommander("arm")
  ## MoveGroup Commander Object for gripper.
  gripper_group = moveit_commander.MoveGroupCommander("gripper")

  ## Action clients to the ExecuteTrajectory action server.
  #arm_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
  arm_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
  arm_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for arm')
  
  # gripper_client = actionlib.SimpleActionClient('execute_trajectory',
  #   moveit_msgs.msg.ExecuteTrajectoryAction)
  # gripper_client.wait_for_server()
  # rospy.loginfo('Execute Trajectory server is available for gripper')

  # gripper_group_joint_values = gripper_group.get_current_joint_values()
  # gripper_group_joint_values[0] = 0.018
  # gripper_group_joint_values[1] = 0.0179
  # gripper_group.set_joint_value_target(gripper_group_joint_values)
  # plan = gripper_group.plan()
  # print("plan executed")
  # rospy.sleep(1)
  # gripper_group.go(wait=True)
  # rospy.sleep(1)
  
  ## Set a named joint configuration as the goal to plan for a move group.
  ## Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
  arm_group.set_named_target("home")
  ## Plan to the desired joint-space goal using the default planner (RRTConnect).
  arm_plan_home = arm_group.plan()
  ## Create a goal message object for the action server.
  #arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  arm_goal = ExecuteTrajectoryGoal()
  ## Update the trajectory in the goal message.
  arm_goal.trajectory = arm_plan_home

  ## Send the goal to the action server.
  arm_client.send_goal(arm_goal)
  arm_client.wait_for_result()  #actions are non blocking 

  return arm_group, 

def gripper_waypoints():
  #Though the 2 groups arm and gripper uses two different clients(plan) separetely they send their goals to the same server execute trajectory goal 
  #so there will be a chance of preempting the previous command. We have to issue a cancel request before we send a new goal to the same server. For that we use another API to compute the straight line motions or cartesian paths. 

  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []
  # start with the current pose
  current_pose = arm_group.get_current_pose()
  rospy.sleep(1)
  current_pose = arm_group.get_current_pose()  # we are calling it again to address the time synchornisations issues

  print("current_pose:",current_pose)
  ## create linear offsets to the current pose
  new_eef_pose = geometry_msgs.msg.Pose()
  print("new_eef_pose", new_eef_pose)


  # Manual offsets because we don't have a camera to detect objects yet.
  new_eef_pose.position.x = current_pose.pose.position.x + 0.10
 #new_eef_pose.position.y = current_pose.pose.position.y - 0.05   
 #new_eef_pose.position.z = current_pose.pose.position.z - 0.05     #0.20

  print("new_eef_x", new_eef_pose.position.x)
  # Retain orientation of the current pose.
  new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

  waypoints.append(new_eef_pose)
  waypoints.append(current_pose.pose)
  # print("new_position",new_eef_pose.position)
  # print("current orientation:",current_pose.pose.position)

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  fraction = 0.0
  for count_cartesian_path in range(0,3):
    if fraction < 1.0:
      (plan_cartesian, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    else:
      break

  arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  arm_goal.trajectory = plan_cartesian
  arm_client.send_goal(arm_goal)
  arm_client.wait_for_result()

  # gripper_group.set_named_target("close")
  # plan2 = gripper_group.go(wait=True)
  # rospy.sleep(1)
  # print("gripper close")

  arm_group.set_named_target("start")
  arm_plan_start = arm_group.plan()
  arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  arm_goal.trajectory = arm_plan_start
  print("planning to start")

  arm_client.send_goal(arm_goal)
  arm_client.wait_for_result()

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

def getPreGraspPosture():
    '''This function calls the pre grasp position for the manipulator to move'''
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "base_link"
    pre_grasp_posture.header.stamp = rospy.Time.now()
    pre_grasp_posture.joint_names = ["gripper","gripper_sub"]
    pos = JointTrajectoryPoint()
    pos.positions.append(0.018)
    print("position", pos.positions)
    #print("pre grasping position")
    #pos.positions.append(0)
    pre_grasp_posture.points.append(pos)   ### needs to check this calling
    return pre_grasp_posture



def createPickupGoal(group="arm", target="part",eef ="gripper", grasp_pose= PoseStamped(), possible_grasps=[]):
    pick = PickupGoal()
    pick.target_name = target
    pick.group_name = group
    pick.end_effector = eef

    pick.possible_grasps.extend(possible_grasps)
    pick.allowed_planning_time = 5.0
    pick.planning_options.planning_scene_diff.robot_state.is_diff = True
    pick.planning_options.planning_scene_diff.is_diff= True
    pick.planning_options.plan_only = False
    pick.planning_options.look_around =False
    pick.planning_options.replan = True
    pick.planning_options.replan_attempts = 10
    pick.planning_options.replan_delay = 5

    return pick


def generategrasps(pose, width=0.04):
    '''Send a request to the Grasp generator service'''

    grasps_ac = actionlib.SimpleActionClient('/grasp_generator_server/generate',GenerateGraspsAction)
    #grasps_ac.wait_for_server()
    rospy.loginfo("Successfully connected")
    goal= GenerateGraspsGoal()
    goal.pose = pose
    goal.width = width
    grasps_ac.send_goal(goal)
    
    t_start = rospy.Time.now()
    #grasps_ac.wait_for_result()
    t_end = rospy.Time.now()
    t_total = t_end - t_start
    rospy.loginfo("Result received in" + str(t_total.to_sec()))
    #grasp_list = grasps_ac.get_result().grasps
    grasp_list = grasps_ac.get_result()
    print("grasp_list", grasp_list)
    return grasp_list

# def CreatePlaceGoal(place_pose, group ="arm", target = "part"):
#     "Using placeGOal to the provided data"
#     place = PlaceGoal()
#     place.group_name = group
#     place.attached_object_name = target

#     place.place_locations = createPlaceLocations(place_pose)
#     place.allowed_planning_time = 10.0
#     place.planning_options.planning_scene_diff.is_diff = True
#     place.planning_options.planning_scene_diff.robot_state.is_diff = True
#     place.planning_options.plan_only = False
#     place.planning_options.replan = True
#     place.planning_options.replan_attempts = 10
#     place.allow_gripper_support_collision = False
#     place.allowed_touch_objects = ['table'] # Sometimes refuses to do the movement if this is not set
    
#     #placeg.planner_id
#     return place

# def createPlaceLocations(posestamped):
#     place_locs = []
#     for yaw_angle in np.arange(0, 2*pi, radians(15)):
#         pl = PlaceLocation()
#         pl.place_pose = posestamped
#         newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
#         pl.place_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
#         pl.pre_place_approach = createGripperTranslation(Vector3(0.0, 0.0, -1.0))
#         pl.post_place_retreat = createGripperTranslation(Vector3(0.0, 0.0, 1.0))
    
#         pl.post_place_posture = getPreGraspPosture()
#         place_locs.append(pl)
        
#     return place_locs

# def createGripperTranslation(direction_vector,desired_distance=0.15,min_distance=0.1):
#     '''Translate the gripper position in a defined direction'''
#     gripper_trans = GripperTranslation()
#     gripper_trans.direction.vector.x = direction_vector.x
#     gripper_trans.direction.vector.y = direction_vector.y
#     gripper_trans.direction.vector.z = direction_vector.z
#     gripper_trans.direction.header.frame_id = "base_link"
#     gripper_trans.direction.header.stamp = rospy.Time.now()
#     gripper_trans.desired_distance = desired_distance
#     gripper_trans.min_distance = min_distance
#     return gripper_trans

if __name__=='__main__':
    rospy.init_node("pick_place")
    #while not rospy.is_shutdown:
    plan_motion()  ## calling the function
    getPreGraspPosture()
    print("starting the pregrasp posture")
  
    rospy.loginfo("Connecting to the pick place node")
    pickup_ac = actionlib.SimpleActionClient('/pickup',PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Connection Succesfull")

    #createPickupGoal()

      # rospy.loginfo("Conencting to place AS")
      # place_ac = actionlib.SimpleActionClient('/place',PlaceAction)
      # place_ac.wait_for_server()
      # rospy.loginfo("Successfully Connected")

    rospy.loginfo("setting up planning interface")
    scene = PlanningSceneInterface()
    rospy.sleep(1)

    rospy.loginfo("Cleaning world objects from possible previous runs")
    scene.remove_world_object("table")
    scene.remove_world_object("part")

      # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.header.stamp = rospy.Time.now()
        

      ##table position
    p.pose.position.x = 0.5
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.0  #0.65
    p.pose.orientation.w = 1.0

    scene.add_box("table", p, (0.5, 1, 0.1))
    rospy.loginfo("Added the table")

      #object position
    p.pose.position.x = 0.1
    p.pose.position.y = -0.2
    p.pose.position.z = 0.5
        
      #these 3 are removed by me
    angle = radians(80) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    p.pose.orientation = Quaternion(*quat.tolist())
        
    scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added part to world")
    
    rospy.sleep(0.1)    
        
    pose_grasp = copy.deepcopy(p)
    #pose = copy.deepcopy(p)

    #print("pose_grasp", pose_grasp)
        
    p.pose.position.x -= 0.10 # 0.25 makes grasp work...
    p.pose.position.y -= 0.04
        
      #possible_grasps = createRandomGrasps(pose_grasp, 1)
      #empty_pose = Pose()
        
    possible_grasps = generategrasps(pose_grasp.pose.position, 0.04) # using grasp generator AS   #removed by me
    #possible_grasps = generategrasps(pose)
    #print("possible_grasps", possible_grasps)   #removed by me
        
      #possible_grasps = createRandomGrasps(pose_grasp.pose, 136)

      #rospy.loginfo("possible grasp locations")   ## modified by me
      #publish_grasps_as_poses(possible_grasps)   ## modifed by me
    goal = createPickupGoal("arm", "part","gripper", pose_grasp, possible_grasps)  ## possible grasps removed by me
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
        
        #scene.remove_world_object("part")
        
    #pickup_ac.wait_for_result()
    result = pickup_ac.get_result()

    rospy.loginfo("Result is:")
        #print result
        #rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))   #changed by me

    p.pose.position.x = 0.1
    p.pose.position.y = 0.2
    p.pose.position.z = 0.3

      # rospy.loginfo("Creating Place Goal")

    gripper_waypoints()   ##waypoints function

    goal = createPlaceGoal(p,"arm")
    rospy.loginfo("Sending goal")
    place_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")

    #place_ac.wait_for_result()
        


