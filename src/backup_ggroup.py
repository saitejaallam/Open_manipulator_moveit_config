#!/usr/bin/env python

#Author: Sai Teja

import sys
import copy
import rospy
from math import pi,radians, sqrt
from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
import actionlib
from math import pi, radians, sqrt
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, ExecuteTrajectoryGoal, ExecuteTrajectoryAction, ExecuteTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('pick_place', anonymous=True)
   
    #Use the planning scene object to add or remove objects
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    arm = MoveGroupCommander("arm")
    gripper = MoveGroupCommander("gripper")
    #eef = arm.get_end_effector_link()
    #print "eef:",eef
    rospy.sleep(2)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    print("plannin frame:", p.header.frame_id)
    #p.header.frame_id = "base_link"

    # add a table
    p.pose.position.x = 0.2 #0.42
    p.pose.position.y = 0 # -0.2
    p.pose.position.z = 0
    scene.add_box("table", p, (0.1, 0.5, 0.1))   #(name, pose, dimensions)

    # add an object to be grasped
    p.pose.position.x = 0.2 #0.205
    p.pose.position.y = 0   #-0.12
    p.pose.position.z = 0.1   #0.7
    scene.add_box("part", p, (0.07, 0.01, 0.1))

    print "table and obj added"
   
    rospy.sleep(1)

    #started the action client to perform the action and wait for sometime to execute the subsequent operation
    #otherwise it throws a warning in the console

    arm_client = actionlib.SimpleActionClient('execute_trajectory',ExecuteTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for arm')
  
    gripper_client = actionlib.SimpleActionClient('execute_trajectory',ExecuteTrajectoryAction)
    gripper_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for gripper')

    gripper_joint_values = gripper.get_current_joint_values()
    gripper_joint_values[0] = 0.018
    gripper_joint_values[1] = 0.0179
    gripper.set_joint_value_target(gripper_joint_values)
    plan = gripper.plan()
    print("gripper opened")
    rospy.sleep(1)
    gripper.go(wait=True)
    rospy.sleep(1)

    arm.set_named_target("home")
    #arm.go()
    #arm.plan()
    arm_goal = ExecuteTrajectoryGoal()
    arm_goal.trajectory = arm.plan()
    #arm.execute(wait=True)

    #rospy.loginfo("reaching home position")

    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result()

    #arm.set_planner_id("RRTConnectkConfigDefault")  #we can set the planning id here for motion planning
   
    # gripper.set_named_target("open")
    # gripper.go()
    # gripper.plan()
    #gripper.execute()

    #rospy.loginfo("gripper opened")
   
    rospy.sleep(1)

     ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
#     waypoints = []
#   # start with the current pose
#     current_pose = arm.get_current_pose()
#     rospy.sleep(1)
#     current_pose = arm.get_current_pose()  # we are calling it again to address the time synchornisations issues

#     print("current_pose:",current_pose)
#   ## create linear offsets to the current pose
#     new_eef_pose = Pose()
#     print("new_eef_pose", new_eef_pose)


#   # Manual offsets because we don't have a camera to detect objects yet.
#     new_eef_pose.position.x = current_pose.pose.position.x + 0.10
#  #new_eef_pose.position.y = current_pose.pose.position.y - 0.05   
#  #new_eef_pose.position.z = current_pose.pose.position.z - 0.05     #0.20

#     print("new_eef_x", new_eef_pose.position.x)
#     print("new_eef_y", new_eef_pose.position.y)
#     print("new_eef_z", new_eef_pose.position.z)
#   # Retain orientation of the current pose.
#     new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

#     waypoints.append(new_eef_pose)
#     waypoints.append(current_pose.pose)
#   # print("new_position",new_eef_pose.position)
#   # print("current orientation:",current_pose.pose.position)

#   ## We want the cartesian path to be interpolated at a resolution of 1 cm
#   ## which is why we will specify 0.01 as the eef_step in cartesian
#   ## translation.  We will specify the jump threshold as 0.0, effectively
#   ## disabling it.
#     fraction = 0.0
#     for count_cartesian_path in range(0,3):
#         if fraction < 1.0:
#             (plan_cartesian, fraction) = arm.compute_cartesian_path(
#                                     waypoints,   # waypoints to follow
#                                     0.01,        # eef_step
#                                     0.0)         # jump_threshold
#         else:
#             break

    grasps = []
   
    g = Grasp()
    g.id = "pick_place"
    grasp_pose = PoseStamped()
    #print("grasp_pose", grasp_pose)
    grasp_pose.header.frame_id = "base_footprint"
    grasp_pose.pose.position.x = 0.2 #0.148554
    grasp_pose.pose.position.y = 0   #-0.116075
    grasp_pose.pose.position.z = 0.1 #0.70493

    angle = radians(80) # angles are expressed in radians
    quat = quaternion_from_euler(-angle/2, -angle/2, -angle/4) # roll, pitch, yaw
    print("quat",quat)
    #grasp_pose.pose.orientation = Quaternion(quat)

    ##The orientation/quaternion values shoudl be normalised for moveit
    grasp_pose.pose.orientation.x = -0.37232056 #-0.709103
    grasp_pose.pose.orientation.y = -0.26070166 #0.0137777
    grasp_pose.pose.orientation.z = -0.26853582 #0.0164031
    grasp_pose.pose.orientation.w = 0.84929415 #0.704779
   
    arm.set_pose_target(grasp_pose)    #Set the pose of the end-effector, if one is available. The expected input is a Pose message, a PoseStamped message or a list of 6 floats:
    arm.go()

    # arm_goal = ExecuteTrajectoryGoal()
    # arm_goal.trajectory = plan_cartesian
    # arm_client.send_goal(arm_goal)
    # arm_client.wait_for_result()

    gripper.set_named_target("close")
    plan2 = gripper.go(wait=True)
    rospy.sleep(1)
    print("gripper close")

    arm.set_named_target("start")
    arm_plan_start = arm.plan()
    arm_goal = ExecuteTrajectoryGoal()
    arm_goal.trajectory = arm_plan_start
    print("planning to start")

    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result()

    # arm_goal1 = ExecuteTrajectoryGoal()
    # arm_goal1.trajectory = arm.plan()
    #arm.execute(wait=True)

    #rospy.loginfo("reaching home position")

    # arm_client.send_goal(arm_goal1)
    # arm_client.wait_for_result()
    #arm.plan()

    rospy.sleep(2)
   
    # set the grasp pose
    #commented out by me to test it

    g.grasp_pose = grasp_pose
   
    # define the pre-grasp approach
    g.pre_grasp_approach.direction.header.frame_id = "base_footprint"  #This should be your gripper tool frame, since you usually want the
                                                                       #approach to happen along a specific axis of the gripper, not the base.

    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0.0
    g.pre_grasp_approach.direction.vector.z = 0.0
    g.pre_grasp_approach.min_distance = 0.05 #0.001
    g.pre_grasp_approach.desired_distance = 0.1
   
    g.pre_grasp_posture.header.frame_id = "base_footprint"
    g.pre_grasp_posture.joint_names = ["gripper"]
   
    pos = JointTrajectoryPoint()
    pos.positions.append(0.018)
   
    g.pre_grasp_posture.points.append(pos)
   
    # set the grasp posture
    g.grasp_posture.header.frame_id = "base_footprint"
    g.grasp_posture.joint_names = ["gripper"]

    pos = JointTrajectoryPoint()
    pos.positions.append(0.0)
    pos.effort.append(0.0)
   
    g.grasp_posture.points.append(pos)   ## needs clarification

    # set the post-grasp retreat
    g.post_grasp_retreat.direction.header.frame_id = "base_footprint"
    g.post_grasp_retreat.direction.vector.x = 0.0
    g.post_grasp_retreat.direction.vector.y = 0.0
    g.post_grasp_retreat.direction.vector.z = 1.0
    g.post_grasp_retreat.desired_distance = 0.25
    g.post_grasp_retreat.min_distance = 0.01

    #g.allowed_touch_objects = ["table"]
    g.allowed_touch_objects = ["part"]

    g.max_contact_force = 0
   
    # append the grasp to the list of grasps
    grasps.append(g)

    rospy.sleep(2)

    #interface = MoveGroupInterface("arm", "base_footprint", "move_group")
    #interface.setPlanningTime(45.0)
    result = False
    n_attempts = 0

    # pick the object
    while result == False:

        result = robot.arm.pick("part", grasps)
        n_attempts +=1
        print "Attempts:", n_attempts
        rospy.sleep(0.2)
    #print("gr:", grasps)
    rospy.loginfo("part found and picking")

    rospy.spin()
    roscpp_shutdown()
   