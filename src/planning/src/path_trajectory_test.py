#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback

from intera_interface import gripper as robot_gripper

from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PositionConstraint

from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
try:
    from controller_pid import Controller
except ImportError:
    pass
    
def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    gripper = robot_gripper.Gripper('right_gripper')
    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    gripper.calibrate()
    rospy.sleep(2.0)
    print('opening')
    gripper.open()
    rospy.sleep(1.0)
    print('opened')
    print('closing')
    gripper.close()
    rospy.sleep(1.0)
    print('closed')

    # planner.add_box_obstacle(size, name, pose)

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper_tip";
    orien_const.header.frame_id = "base";
    orien_const.orientation.x = 1.0;
    orien_const.orientation.y = 0.0;
    orien_const.orientation.z = 0.0;
    orien_const.absolute_x_axis_tolerance = 0.2;
    orien_const.absolute_y_axis_tolerance = 0.2;
    orien_const.absolute_z_axis_tolerance = 0.2;
    orien_const.weight = 1.0;

    orien_const_2 = PositionConstraint()
    orien_const_2.link_name = "right_gripper_tip";
    orien_const_2.header.frame_id = "base";
    orien_const_2.target_point_offset.x = 0.6;
    orien_const_2.target_point_offset.y = -0.3;
    orien_const_2.target_point_offset.z = 0.0;
    orien_const_2.weight = 1.0;


    with open('/home/cc/ee106a/fl21/class/ee106a-aak/final/src/planning/src/trajectory.txt') as trajectory_txt:
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

        for trans, quat in zip(trans_list, quat_list):
            while not rospy.is_shutdown():
                try:
                    goal_2 = PoseStamped()
                    goal_2.header.frame_id = "base"

                    # import pdb; pdb.set_trace()

                    #x, y, and z position
                    goal_2.pose.position.x = float(trans[0])
                    goal_2.pose.position.y = float(trans[1])
                    goal_2.pose.position.z = float(trans[2])

                    #Orientation as a quaternion
                    goal_2.pose.orientation.x = float(quat[0])
                    goal_2.pose.orientation.y = float(quat[1])
                    goal_2.pose.orientation.z = float(quat[2])
                    goal_2.pose.orientation.w = float(quat[3])

                    plan = planner.plan_to_pose(goal_2, [], [])

                    raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
                except Exception as e:
                    print e
                else:
                    break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
