# T_robot_to_world:
# - Translation: [0.906, -0.101, -0.066]
# - Rotation: in Quaternion [0.999, -0.031, -0.013, -0.015]
#             in RPY (radian) [-3.112, 0.026, -0.062]
#             in RPY (degree) [-178.291, 1.513, -3.563]

# # to get this 
# roslaunch realsense2_camera/launch/rs_rgbd.launch
# roslaunch camera_world ar_track.launch
# rviz 
# 1) get out /visualization_marker
# rosrun tf tf_echo /camera_depth_optical_frame /ar_marker_3
# T_camera_world 
# - Translation: [0.088, -0.044, 0.671]
# - Rotation: in Quaternion [0.694, 0.595, -0.327, 0.239]
#             in RPY (radian) [-3.056, 0.831, 1.455]
#             in RPY (degree) [-175.084, 47.613, 83.378]
# - Translation: [0.151, 0.193, 0.700]
# - Rotation: in Quaternion [0.623, 0.541, -0.370, 0.428]
#             in RPY (radian) [2.788, 1.176, 1.192]
#             in RPY (degree) [159.752, 67.373, 68.309]


from autolab_core import RigidTransform
import numpy as np

# T_robot_world 
trans1 = np.asarray([0.737, 0.011, -0.2])
quat1 = np.asarray([1.000, -0.000, -0.000, 0.000]) #(wxyz)
quat1 = quat1 / np.linalg.norm(quat1)

rot1 = RigidTransform.rotation_from_quaternion(quat1)
T_world_robot = RigidTransform(rotation=rot1, translation=trans1, from_frame="world", to_frame="robot")
# import pdb; pdb.set_trace()

# T_camera_world 
trans2 = np.asarray([0.151, 0.193, 0.700])
quat2 = np.asarray([0.428, 0.623, 0.541, -0.370,]) #(wxyz)
quat2 = quat2 / np.linalg.norm(quat2)

rot2 = RigidTransform.rotation_from_quaternion(quat2)
T_world_camera = RigidTransform(rotation=rot2, translation=trans2, from_frame="world", to_frame="camera")


