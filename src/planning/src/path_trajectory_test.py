#!/usr/bin/env python

import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg

from intera_interface import gripper as robot_gripper

from test_realsense import get_robot_pointcloud
from utils import open_gripper, close_gripper, concatenate_plan
import traceback

from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from controller_pid import Controller

import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg as geom
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from camera_matrix import neutral_joint_state

from level_detection import get_percent_liquid, get_marker_edge, show

import cv2

rospy.wait_for_service('compute_ik')
rospy.init_node('service_query')
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
arm = 'right'

# If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
link = "right_gripper_tip"

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

planner = PathPlanner("right_arm")

Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

point_cloud_robot, cup_left_top_pt_right_robot, cup_right_top_pt_right_robot, webcam_im = get_robot_pointcloud()
target_position_1 = cup_left_top_pt_right_robot.data
target_position_2 = cup_right_top_pt_right_robot.data

webcam_im = webcam_im.data

left, right = None, None
bottom = None
if left is None or right is None or bottom is None:
    print(webcam_im.shape)
    show(webcam_im)
    left = int(raw_input("Left edge index"))
    right = int(raw_input("right edge index"))
    bottom = int(raw_input("bottom edge index"))
top = get_marker_edge(webcam_im, left, right, bottom, vis=False)

LIQUID_THRESHOLD = 0.8
stop_condition = lambda img: (get_percent_liquid(img, left, right, top, bottom, vis=False) > LIQUID_THRESHOLD)

controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))

# Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
right_gripper.calibrate()
rospy.sleep(1.0)

# Open the right gripper
print('Opening...')
right_gripper.open()
rospy.sleep(1.0)
print('Done!')

def move_to_point_ik(group, target_position, target_orientation=[1.0, 0.0, 0.0, 0.0]):
    try:
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        #x, y, and z position
        request.ik_request.pose_stamped.pose.position.x = target_position[0]
        request.ik_request.pose_stamped.pose.position.y = target_position[1]
        request.ik_request.pose_stamped.pose.position.z = target_position[2]

        #Orientation as a quaternion
        request.ik_request.pose_stamped.pose.orientation.x = target_orientation[0]
        request.ik_request.pose_stamped.pose.orientation.y = target_orientation[1]
        request.ik_request.pose_stamped.pose.orientation.z = target_orientation[2]
        request.ik_request.pose_stamped.pose.orientation.w = target_orientation[3]

        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        print(response)

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK and execute
        group.go()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit()

def move_to_point_pid(group, target_position, target_orientation=[1.0, 0.0, 0.0, 0.0], orien_const=None):
    while not rospy.is_shutdown():
        try:
            x, y, z = target_position
            goal_1 = PoseStamped()
            goal_1.header.frame_id = "base"

            #x, y, and z position
            goal_1.pose.position.x = x
            goal_1.pose.position.y = y
            goal_1.pose.position.z = z

            #Orientation as a quaternion
            x, y, z, w = target_orientation
            goal_1.pose.orientation.x = x
            goal_1.pose.orientation.y = y
            goal_1.pose.orientation.z = z
            goal_1.pose.orientation.w = w

            if orien_const:
                plan = planner.plan_to_pose(goal_1, [orien_const], [])
            else:
                # import pdb; pdb.set_trace()
                plan = planner.plan_to_pose(goal_1, [], [])

            if not controller.execute_path(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def move_to_point_sequence(group, target_positions, target_orientations):
    while not rospy.is_shutdown():
        try:
            assert len(target_positions) == len(target_orientations)

            waypoints = []

            for i, (target_position, target_orientation) in enumerate(zip(target_positions, target_orientations)):
                x, y, z = target_position
                goal_1 = geom.Pose()

                #x, y, and z position
                goal_1.position.x = x
                goal_1.position.y = y
                goal_1.position.z = z

                #Orientation as a quaternion
                x, y, z, w = target_orientation
                goal_1.orientation.x = x
                goal_1.orientation.y = y
                goal_1.orientation.z = z
                goal_1.orientation.w = w

                waypoints.append(goal_1)
            
            # import pdb; pdb.set_trace()

            (full_plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)

            # plans = []
            # for i, (target_position, target_orientation) in enumerate(zip(target_positions, target_orientations)):
            #     x, y, z = target_position
            #     goal_1 = PoseStamped()
            #     goal_1.header.frame_id = "base"

            #     #x, y, and z position
            #     goal_1.pose.position.x = x
            #     goal_1.pose.position.y = y
            #     goal_1.pose.position.z = z

            #     #Orientation as a quaternion
            #     x, y, z, w = target_orientation
            #     goal_1.pose.orientation.x = x
            #     goal_1.pose.orientation.y = y
            #     goal_1.pose.orientation.z = z
            #     goal_1.pose.orientation.w = w

            #     # Might have to edit this . . . 
            #     plan = planner.plan_to_pose(goal_1, [], [])
            #     plans.append(plan)

            #     print(i)
            
            # assert len(plans) == len(target_positions)
            
            # full_plan = concatenate_plan(plans)

            # print(group.get_current_pose("right_gripper_tip").pose)

            moveit_robot_state = RobotState()
            joint_state = JointState()
            joint_state.name = group.get_active_joints()
            joint_state.position = group.get_current_joint_values()
            moveit_robot_state.joint_state = joint_state
            traj = group.retime_trajectory(moveit_robot_state, full_plan, 0.1)

            print("FRACTION", fraction)

            execution_result = controller.execute_path(traj, stop_condition=stop_condition)

            if not execution_result:
                raise Exception("Execution failed")
    
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break



def main(robo):

    group = MoveGroupCommander("right_arm")
    move_to_point = move_to_point_pid

    print("go to neutral position")
    group.go(neutral_joint_state)
    print("arrived at neutral position")

    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = 1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    point_up_cup_left = target_position_1.copy() + np.array([0, 0, 0.1])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0])

    point_up_cup_left = target_position_1.copy() + np.array([0, 0, -0.07])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0],) # orien_const=orien_const)

    close_gripper(right_gripper)

    point_up_cup_left = target_position_1.copy() + np.array([0, 0, 0.12])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)

    point_up_cup_left = target_position_2.copy() + np.array([0, 0, 0.12])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)

    with open('/home/cc/ee106a/fl21/class/ee106a-aak/final/src/planning/src/traj3.txt') as trajectory_txt:
        lines = trajectory_txt.readlines()
        lines = [line.rstrip() for line in lines]

        trans_list = []
        quat_list = []

        for line in lines:
            if line.startswith("- Translation: "):
                trans = line.split('[')[1][:-1]
                trans = trans.split(', ')
                trans_list.append(trans)

            if line.startswith("- Rotation: in Quaternion "):
                quat = line.split('[')[1][:-1]
                quat = quat.split(', ')
                quat_list.append(quat)

        # import pdb; pdb.set_trace()
        print(target_position_2)
        offset_x = target_position_2[0] - float(trans_list[0][0])
        offset_y = target_position_2[1] - float(trans_list[0][1])
        offset_z = 0.04

        new_trans_list = []
        new_quat_list = []

        for i, (trans, quat) in enumerate(zip(trans_list, quat_list)):

            x,y,z,w = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            norm = (x ** 2 + y ** 2 + z ** 2 + w ** 2) ** 0.5
            x,y,z,w = x/norm, y/norm, z/norm, w/norm

            new_trans_list.append([float(trans[0]) + offset_x, float(trans[1]) + offset_y, float(trans[2]) + offset_z])
            new_quat_list.append([x, y, z, w])

        move_to_point_sequence(group, new_trans_list, new_quat_list)

        x, y, z, w = float(quat_list[0][0]), float(quat_list[0][1]), float(quat_list[0][2]), float(quat_list[0][3])
        norm = (x ** 2 + y ** 2 + z ** 2 + w ** 2) ** 0.5
        x,y,z,w = x/norm, y/norm, z/norm, w/norm

        move_to_point(group, [float(trans_list[0][0]) + offset_x, float(trans_list[0][1]) + offset_y, float(trans_list[0][2]) + offset_z], [x, y, z, w])


        # rev_trans_list = []
        # rev_quat_list = []

        # for i, (trans, quat) in enumerate(zip(trans_list[::-1][1:], quat_list[::-1][1:])):

        #     x,y,z,w = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        #     norm = (x ** 2 + y ** 2 + z ** 2 + w ** 2) ** 0.5
        #     x,y,z,w = x/norm, y/norm, z/norm, w/norm

        #     rev_trans_list.append([float(trans[0]) + offset_x, float(trans[1]) + offset_y, float(trans[2]) + offset_z])
        #     rev_quat_list.append([x, y, z, w])

        #     # move_to_point(group, [float(trans[0]) + offset_x, float(trans[1]) + offset_y, float(trans[2]) + offset_z], [x, y, z, w])
        
        # move_to_point_sequence(group, rev_trans_list, rev_quat_list)


    point_up_cup_left = target_position_2.copy() + np.array([0, 0, 0.12])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)

    point_up_cup_left = target_position_1.copy() + np.array([0, 0, 0.12])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)

    point_up_cup_left = target_position_1.copy() + np.array([0, 0, -0.07])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)

    open_gripper(right_gripper)
    
    point_up_cup_left = target_position_1.copy() + np.array([0, 0, 0.15])
    move_to_point(group, point_up_cup_left, [1.0, 0.0, 0.0, 0.0], ) # orien_const=orien_const)


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])
