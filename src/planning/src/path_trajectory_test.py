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
import sys

if sys.argv[1] == 'sawyer':
    from intera_interface import gripper as robot_gripper
else:
    from baxter_interface import gripper as robot_gripper

from test_realsense import main as get_robot_pointcloud

def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
        arm = 'right'

    raw_input('Press [ Enter ]: ')
    
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = arm + "_gripper"
    if robo == 'sawyer':
        link += '_tip'

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Open the right gripper
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)
    print('Done!')

    # CV Pipeline
    # Should return the position of the cup in robot frame

    point_cloud_robot = get_robot_pointcloud()
    target_position = point_cloud_robot[:][153600]

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

        for i, (trans, quat) in enumerate(zip(trans_list, quat_list)):
            if i == 0:
                offset_x = target_position.x - float(trans[0])
                offset_y = target_position.y - float(trans[1])
            try:
                #x, y, and z position
                request.ik_request.pose_stamped.pose.position.x = float(trans[0]) + offset_x
                request.ik_request.pose_stamped.pose.position.y = float(trans[1]) + offset_y
                request.ik_request.pose_stamped.pose.position.z = float(trans[2])

                x,y,z,w = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                norm = (x ** 2 + y ** 2 + z ** 2 + w ** 2) ** 0.5
                x,y,z,w = x/norm, y/norm, z/norm, w/norm
                print(x, y, z, w)

                #Orientation as a quaternion
                request.ik_request.pose_stamped.pose.orientation.x = x
                request.ik_request.pose_stamped.pose.orientation.y = y
                request.ik_request.pose_stamped.pose.orientation.z = z
                request.ik_request.pose_stamped.pose.orientation.w = w

                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                print(response)
                group = MoveGroupCommander(arm + "_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                ###group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK and execute
                group.go()
                
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                sys.exit()

            # Close the right gripper
            rospy.sleep(0.1)
            if i == 0:
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])
