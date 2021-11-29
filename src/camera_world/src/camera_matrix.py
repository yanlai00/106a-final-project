# T_robot_to_world:
# At time 1638141365.910
# - Translation: [0.551, -0.578, -0.194]
# - Rotation: in Quaternion [0.997, 0.080, -0.002, 0.014]
#             in RPY (radian) [3.114, 0.005, 0.160]
#             in RPY (degree) [178.394, 0.311, 9.191]


# # to get this 
# roslaunch realsense2_camera/launch/rs_rgbd.launch
# roslaunch camera_world ar_track.launch
# rviz 
# 1) get out /visualization_marker
# rosrun tf tf_echo /camera_link /ar_marker_17
# T_camera_world 
# - Translation: [0.980, -0.176, 0.213]
# - Rotation: in Quaternion [0.347, -0.401, -0.578, 0.620]
#             in RPY (radian) [1.116, -0.095, -1.561]
#             in RPY (degree) [63.935, -5.439, -89.438]


from autolab_core import RigidTransform 

# T_robot_world 
trans = np.asarray([0.551, -0.578, -0.194])
quat = np.asarray([[0.997, 0.080, -0.002, 0.014]])

rot = RigidTransform.rotation_from_quaternion(quat)
T_robot_world = RigidTransform(rotation=rot, translation=trans, from_frame="robot", to_frame="world")

# T_camera_world 
trans = [0.980, -0.176, 0.213]
quat = [0.347, -0.401, -0.578, 0.620]
rot = RigidTransform.rotation_from_quaternion(quat)
T_robot_world = RigidTransform(rotation=rot, translation=trans, from_frame="camera", to_frame="world")
