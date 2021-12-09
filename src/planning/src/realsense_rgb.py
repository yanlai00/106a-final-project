import pyrealsense2 as rs
import matplotlib.pyplot as plt

from perception import RgbdSensorFactory
from camera_matrix import T_world_camera, T_world_robot
import numpy as np
import autolab_core
import pptk
from sklearn.cluster import KMeans

def discover_cams():
    """Returns a list of the ids of all cameras connected via USB."""
    ctx = rs.context()
    ctx_devs = list(ctx.query_devices())
    ids = []
    for i in range(ctx.devices.size()):
        ids.append(ctx_devs[i].get_info(rs.camera_info.serial_number))
    return ids


def get_robot_pointcloud(show_imgs=False):
    ids = discover_cams()
    assert ids, "[!] No camera detected."

    cfg = {}
    cfg["cam_id"] = ids[0]
    cfg["filter_depth"] = True
    cfg["frame"] = "camera"

    sensor = RgbdSensorFactory.sensor("realsense", cfg)
    sensor.start()
    camera_intr = sensor.color_intrinsics

    for _ in range(100):
        color_im, depth_im = sensor.frames()

    color_im_data = color_im.data

    if show_imgs:
        _, axes = plt.subplots(1, 1)
        for ax, im in zip(axes, [color_im_data]):
            ax.imshow(im)
            ax.axis("off")
        plt.show()
    
    edges = np.zeros_like(color_im_data)
    mask_entries = np.where(img[:, :, 0]<70) # code with water
    edges[mask_entries] = 1
    if vis:
        show(edges, bottom_pt)
        show(edges[bottom_pt[0]:,:])

    sensor.stop()
    webcam_sensor.stop()

    return point_cloud_robot, cup_left_top_pt_right_robot, cup_right_top_pt_right_robot, webcam_im

if __name__ == "__main__":
    get_robot_pointcloud(show_imgs=True)
