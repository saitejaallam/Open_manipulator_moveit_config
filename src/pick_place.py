#! /usr/bin/env python

#Author: Sai Teja

import rospy
import actionlib
import copy
import random
from std_msgs.msg import Header

from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Twist, PoseStamped, Pose,PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from moveit_msgs.msg import Grasp,GripperTranslation,PickupAction,PlaceAction,MoveItErrorCodes,PickupGoal
from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal,GenerateGraspsResult
from math import pi,radians
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def getPreGraspPosture():
    '''This function calls the pre grasp position for the manipulator to move'''
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "base_link"
    pre_grasp_posture.header.stamp = rospy.Time.now()
    pre_grasp_posture.joint_names = ["gripper","gripper_sub"]
    pos = JointTrajectoryPoint()
    pos.positions.append(0.18)
    print("position", pos.positions)
    #print("pre grasping position")
    #pos.positions.append(0)
    pre_grasp_posture.points.append(pos)
    return pre_grasp_posture

def getGraspPosture():  ## grasp position with gripper closed
    '''Attains the grasping posture'''
    grasp_posture = JointTrajectory()
    grasp_posture.header.frame_id = "base_link"
    grasp_posture.header.stamp = rospy.Time.now()
    grasp_posture.joint_names = ["gripper","gripper_sub"]
    pos = JointTrajectory()
    pos.positions.append(0.01)

    print("grasping posture")
    #pos.positions.append()
    grasp_posture.points.append(pos)
    return getGraspPosture

def createGripperTranslation(direction_vector,desired_distance=0.15,min_distance=0.1):
    '''Translate the gripper position in a defined direction'''
    gripper_trans = GripperTranslation()
    gripper_trans.direction.vector.x = direction_vector.x
    gripper_trans.direction.vector.y = direction_vector.y
    gripper_trans.direction.vector.z = direction_vector.z
    gripper_trans.direction.header.frame_id = "base_link"
    gripper_trans.direction.header.stamp = rospy.Time.now()
    gripper_trans.desired_distance = desired_distance
    gripper_trans.min_distance = min_distance
    return gripper_trans

def add_offset_om_arm(grasp_pose):
    rospy.loginfo("Adding offset for OM arm")
    offset_grasp_pose = PoseStamped(grasp_pose.header, copy.deepcopy(grasp_pose.pose))
#     offset_grasp_pose.pose.position.x += -0.054 # X on arm 7 link is -Y in base_link
#     offset_grasp_pose.pose.position.y += -0.018 * 2 # Y on arm 7 link is -Z in base link
#     offset_grasp_pose.pose.position.z += 0.0 # Z on arm 7 link is X in base link
    
    offset_grasp_pose.pose.position.x += -0.054 - 0.018 # X on arm 7 link is -Y in base_link
    offset_grasp_pose.pose.position.y += -0.018 * 2  # Y on arm 7 link is -Z in base link 
    offset_grasp_pose.pose.position.z += -0.018 # Z on arm 7 link is X in base link
    roll, pitch, yaw = transformations.euler_from_quaternion([offset_grasp_pose.pose.orientation.x,
                                                             offset_grasp_pose.pose.orientation.y,
                                                             offset_grasp_pose.pose.orientation.z,
                                                             offset_grasp_pose.pose.orientation.w])
    newroll = roll + pi # putting the hand in the inverse position
    newquat = transformations.quaternion_from_euler(newroll, pitch, yaw)
    offset_grasp_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
    return offset_grasp_pose


def createGrasp(grasp_pose, allowed_touch_objects=[], pre_grasp_posture=None, grasp_posture=None, pre_grasp_approach=None, post_grasp_retreat=None, id_grasp="grasp_"):
    '''Creating grasp position'''
    grasp = Grasp()
    grasp.id = id_grasp

    grasp_pose_with_offset = add_offset_om_arm(grasp_pose)
    grasp.grasp_pose = grasp_pose_with_offset

    if pre_grasp_posture ==None:
        grasp.pre_grasp_posture = getPreGraspPosture
    else:
        grasp.pre_grasp_posture = pre_grasp_posture

    if grasp_posture ==None:
        grasp.grasp_posture = getGraspPosture
    else:
        grasp.grasp_posture = getGraspPosture

    grasp.allowed_touch_objects = allowed_touch_objects
    grasp.max_contact_force = 0 # below 0 disables the force imposed by gripper on the object

    if pre_grasp_approach != None:
        grasp.pre_grasp_approach = pre_grasp_approach
    if post_grasp_retreat !=None:  
        grasp.post_grasp_retreat = post_grasp_retreat         #The retreat motion to perform when releasing the object; this information
# is not necessary for the grasp itself, but when releasing the object,
# the information will be necessary. The grasp used to perform a pickup
# is returned as part of the result, so this information is available for 
# later use.

    return grasp

def createRandomGrasps(grasp_pose, num_grasps=1):
    """ Returns a list of num_grasps Grasp's around grasp_pose """
    list_grasps = []
    for grasp_num in range(num_grasps):
        my_pre_grasp_approach = createGripperTranslation(Vector3(1.0, 0.0, 0.0)) # rotate over here?
        my_post_grasp_retreat = createGripperTranslation(Vector3(0.0, 0.0, 1.0))
        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        grasp_pose_copy = copy.deepcopy(grasp_pose)
        modified_grasp_pose = PoseStamped(header, grasp_pose_copy)
        #modified_grasp_pose.pose.position.x -= random.random() * 0.25 # randomize entrance to grasp in 25cm in x
        #modified_grasp_pose.pose.position.y -= random.random() * 0.10 # randomize entrance to grasp in 10cm in y
        #modified_grasp_pose.pose.orientation.z = -1.0
        
        grasp = createGrasp(modified_grasp_pose, # change grasp pose?
                    allowed_touch_objects=["part"], 
                    pre_grasp_posture=getPreGraspPosture(),
                    grasp_posture=getPreGraspPosture(),
                    pre_grasp_approach=my_pre_grasp_approach,
                    post_grasp_retreat=my_post_grasp_retreat,
                    id_grasp="random_grasp_" + str(grasp_num)
                    )
        list_grasps.append(grasp)
    return list_grasps
    
def createPickupGoal(group="arm", target="part", grasp_pose= PoseStamped()):
    pick = PickupGoal()
    pick.target_name = target
    pick.group_name = group

    #pick.possible_grasps.extend(possible_grasps)
    pick.allowed_planning_time = 5.0
    pick.planning_options.planning_scene_diff.robot_state.is_diff = True
    pick.planning_options.planning_scene_diff.is_diff= True
    pick.planning_options.plan_only = False
    pick.planning_options.look_around =False
    pick.planning_options.replan = True
    pick.planning_options.replan_attempts = 10
    pick.planning_options.replan_delay = 5

    return pick

def generate_grasps(pose, width=0.04):
    '''Send a request to the Grasp generator service'''

    grasps_ac = actionlib.SimpleActionClient('/grasp_generator_server/generate',GenerateGraspsAction)
    grasps_ac.wait_for_server()
    rospy.loginfo("Successfully connected")
    goal= GenerateGraspsGoal()
    goal.pose = pose
    goal.width = width
    grasps_ac.send_goal(goal)

    t_start = rospy.Time.now()
    grasps_ac.wait_for_result()
    t_end = rospy.Time.now()
    t_final = t_end - t_start
    #grasp_list = grasps_ac.get_result().grasps
    grasp_list = grasps_ac.get_result()
    return grasp_list

def publish_grasps_as_poses(grasps):

    grasp_publisher = rospy.Publisher("grasp_pose_from_block_bla", PoseArray)
    grasp_msg = Grasp()
    grasp_PA = PoseArray()
    header = Header()
    header.frame_id ="base_link"
    header.stamp = rospy.Time.now()
    grasp_PA.header = header
    for grasp_msg in grasps:
        print grasp_msg
        p = Pose(grasp_msg.grasp_pose.pose.position,graspmsg.grasp_pose.pose.orientation)
        grasp_PA.poses.append(p)
    grasp_publisher.publish(grasp_PA)
    

if __name__=='__main__':
    rospy.init_node("pick_place")

    rospy.loginfo("Connecting to the pick place node")
    pickup_ac = actionlib.SimpleActionClient('/pickup',PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Connection Succesfull")

    rospy.loginfo("Conencting to place AS")
    place_ac = actionlib.SimpleActionClient('/place',PlaceAction)
    place_ac.wait_for_server()
    rospy.loginfo("Successfully Connected")

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
    p.pose.position.x = 0.3
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.0  #0.65
    p.pose.orientation.w = 1.0

    scene.add_box("table", p, (0.5, 1.5, 0.1))
    rospy.loginfo("Added the table")

    #object position
    p.pose.position.x = 0.1
    p.pose.position.y = -0.2
    p.pose.position.z = 0.5
    
    #these 3 are removed by me
    #angle = radians(80) # angles are expressed in radians
    #quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    #p.pose.orientation = Quaternion(*quat.tolist())
    
    scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added part to world")
  
    rospy.sleep(0.1)    
    
    pose_grasp = copy.deepcopy(p)

    print("pose_grasp", pose_grasp)
    
    #pose_grasp.pose.position.x -= 0.10 # 0.25 makes grasp work...
    #pose_grasp.pose.position.y -= 0.04
    
    #possible_grasps = createRandomGrasps(pose_grasp, 1)
    #empty_pose = Pose()
    
    possible_grasps = generate_grasps(pose_grasp.pose) # using grasp generator AS   #removed by me
    #print("possible_grasps", possible_grasps)   #removed by me
    
    #possible_grasps = createRandomGrasps(pose_grasp.pose, 136)

    #rospy.loginfo("possible grasp locations")   ## modified by me
    #publish_grasps_as_poses(possible_grasps)   ## modifed by me
    goal = createPickupGoal("arm", "part", pose_grasp, possible_grasps)  ## possible grasps removed by me
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
    
    #scene.remove_world_object("part")
    
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()

    rospy.loginfo("Result is:")
    #print result
    #rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))   #changed by me

    p.pose.position.x = 0.1
    p.pose.position.y = 0.2
    p.pose.position.z = 0.3

    rospy.loginfo("Creating Place Goal")
    
