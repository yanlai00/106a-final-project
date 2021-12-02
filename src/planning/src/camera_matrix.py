# T_robot_to_world:
# At time 1638141365.910
# - Translation: [0.551, -0.578, -0.194]
# - Rotation: in Quaternion [0.997, 0.080, -0.002, 0.014]
#             in RPY (radian) [3.114, 0.005, 0.160]
#             in RPY (degree) [178.394, 0.311, 9.191]

# At time 1638415335.773
# - Translation: [0.747, 0.114, -0.270]
# - Rotation: in Quaternion [0.999, 0.012, -0.024, 0.019]
#             in RPY (radian) [3.104, 0.049, 0.023]
#             in RPY (degree) [177.869, 2.785, 1.338]

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

# - Translation: [1.113, -0.276, -0.172]
# - Rotation: in Quaternion [-0.175, -0.221, 0.544, 0.790]
#             in RPY (radian) [-0.551, -0.159, 1.251]
#             in RPY (degree) [-31.592, -9.110, 71.682]



from autolab_core import RigidTransform
import numpy as np

# T_robot_world 
trans = np.asarray([0.747, 0.114, -0.270])
quat = np.asarray([0.999, 0.012, -0.024, 0.019])

rot = RigidTransform.rotation_from_quaternion(quat)
T_robot_world = RigidTransform(rotation=rot, translation=trans, from_frame="robot", to_frame="world")

# T_camera_world 
trans = np.asarray([1.113, -0.276, -0.172])
quat = np.asarray([-0.175, -0.221, 0.544, 0.790])
rot = RigidTransform.rotation_from_quaternion(quat)
T_camera_world = RigidTransform(rotation=rot, translation=trans, from_frame="camera", to_frame="world")


