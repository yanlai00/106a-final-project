#!/usr/bin/env python
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

def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'

    gripper = robot_gripper.Gripper(arm)
    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    gripper.calibrate()
    rospy.sleep(2.0)

    while not rospy.is_shutdown():
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
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.770
        request.ik_request.pose_stamped.pose.position.y = -0.197
        request.ik_request.pose_stamped.pose.position.z = 0.359      
        request.ik_request.pose_stamped.pose.orientation.x = 1.0
        request.ik_request.pose_stamped.pose.orientation.y = 0.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


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
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.770
        request.ik_request.pose_stamped.pose.position.y = -0.197
        request.ik_request.pose_stamped.pose.position.z = 0.159      
        request.ik_request.pose_stamped.pose.orientation.x = 1.0
        request.ik_request.pose_stamped.pose.orientation.y = 0.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_position_target([0.774, -0.191, -0.154])

            # Plan IK and execute
            group.go()

            group.set_pose_target(request.ik_request.pose_stamped)
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



    
        # Close the right gripper
        print('Closing...')
        gripper.close()
        rospy.sleep(1.0)
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.644
        request.ik_request.pose_stamped.pose.position.y = -0.597
        request.ik_request.pose_stamped.pose.position.z = -0.039      
        request.ik_request.pose_stamped.pose.orientation.x = 0.740
        request.ik_request.pose_stamped.pose.orientation.y = -0.017 # 0.707
        request.ik_request.pose_stamped.pose.orientation.z = 0.671
        request.ik_request.pose_stamped.pose.orientation.w = 0.049 # 0.707
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # Open the right gripper
        print('Opening...')
        gripper.open()
        rospy.sleep(1.0)
        print('Done!')


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

