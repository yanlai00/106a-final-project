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
# - Translation: [0.156, 0.188, 0.697]
# - Rotation: in Quaternion [0.590, 0.585, -0.413, 0.373]
#             in RPY (radian) [-3.030, 1.178, 1.637]
#             in RPY (degree) [-173.604, 67.494, 93.818
# - Translation: [0.094, 0.185, 0.753]
# - Rotation: in Quaternion [0.603, 0.572, -0.410, 0.375]
#             in RPY (radian) [-3.097, 1.179, 1.547]
#             in RPY (degree) [-177.472, 67.526, 88.632]
# 
# position: Neutral joint states
# [0.3240888671875, -0.3903671875, 0.610984375, -0.042935546875, -1.4750556640625, -2.976302734375, -2.3462490234375, 1.5772646484375, 0.0]



from autolab_core import RigidTransform
import numpy as np

# T_robot_world 
# trans1 = np.asarray([0.811, -0.084, -0.2])
trans1 = np.asarray([0.84, -0.2, -0.215])
quat1 = np.asarray([1.000, -0.000, -0.000, 0.000]) #(wxyz)
quat1 = quat1 / np.linalg.norm(quat1)

rot1 = RigidTransform.rotation_from_quaternion(quat1)
T_world_robot = RigidTransform(rotation=rot1, translation=trans1, from_frame="world", to_frame="robot")
# import pdb; pdb.set_trace()

# T_camera_world 
trans2 = np.asarray([0.094, 0.185, 0.753])
quat2 = np.asarray([0.375, 0.603, 0.572, -0.410]) #(wxyz)
quat2 = quat2 / np.linalg.norm(quat2)

rot2 = RigidTransform.rotation_from_quaternion(quat2)
T_world_camera = RigidTransform(rotation=rot2, translation=trans2, from_frame="world", to_frame="camera")

neutral_joint_state = np.array([-0.119646484375, -0.2901474609375, -0.2213916015625, 0.929955078125, -2.7966474609375, -0.955185546875, 1.5758310546875])
