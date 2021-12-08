#!/usr/bin/env python

import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

from intera_interface import Limb

import traceback

from moveit_msgs.msg import OrientationConstraint

from path_planner import PathPlanner

# from controller import Controller
# from controller_pd import Controller
from controller_pid import Controller

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from intera_interface import gripper as robot_gripper
from test_realsense import get_robot_pointcloud

import cv2

def close_gripper(gripper):
    try:
        print('Closing...')
        gripper.close()
        rospy.sleep(1.0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit()

def open_gripper(gripper):
    try:
        print('Opening...')
        gripper.open()
        rospy.sleep(1.0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit()

def concatenate_plan(plans):
    full_plan = RobotTrajectory()
    full_plan.joint_trajectory.joint_names = plans[0].joint_trajectory.joint_names

    full_positions = []
    full_veloities = []
    full_accelerations = []

    for plan in plans:
        for tstep in range(len(plan.joint_trajectory.points)):
            full_positions.append(plan.joint_trajectory.points[tstep].positions)
            if tstep == 0:
                full_veloities.append(plan.joint_trajectory.points[1].velocities)
            elif tstep == len(plan.joint_trajectory.points) - 1:
                full_veloities.append(plan.joint_trajectory.points[-2].velocities)
            else:
                full_veloities.append(plan.joint_trajectory.points[tstep].velocities)
            full_accelerations.append(plan.joint_trajectory.points[tstep].accelerations)
    
    full_veloities[0] = plans[0].joint_trajectory.points[0].velocities
    full_veloities[-1] = plans[-1].joint_trajectory.points[-1].velocities

    for tstep in range(len(full_veloities)):
        point = JointTrajectoryPoint()
        point.positions = full_positions[tstep]
        point.velocities = full_veloities[tstep]
        point.accelerations = full_accelerations[tstep]
        full_plan.joint_trajectory.points.append(point)
    
    return full_plan

# def reverse_plan(plan):
#     full_plan = RobotTrajectory()
#     full_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names

#     full_positions = []
#     full_veloities = []
#     full_accelerations = []


#     for tstep in range(len(plan.joint_trajectory.points)):
#         full_positions.append(plan.joint_trajectory.points[tstep].positions)
#         full_veloities.append((-1 * np.array(plan.joint_trajectory.points[tstep].velocities).tolist()))
#         full_accelerations.append((-1 * np.array(plan.joint_trajectory.points[tstep].accelerations).tolist()))
    
#     full_positions = full_positions[::-1]
#     full_veloities = full_veloities[::-1]
#     full_accelerations = full_accelerations[::-1]

#     for tstep in range(len(full_veloities)):
#         point = JointTrajectoryPoint()
#         point.positions = full_positions[tstep]
#         point.velocities = full_veloities[tstep]
#         point.accelerations = full_accelerations[tstep]
#         full_plan.joint_trajectory.points.append(point)
    
#     return full_plan

# def find_closest_point(group, trans_list, quat_list):
#     closest_idx = 0
#     assert len(trans_list) == len(quat_list)
#     for i in range(len(trans_list)):
#         trans = np.array(trans_list[i])
#         quat = np.array(quat_list[i])


