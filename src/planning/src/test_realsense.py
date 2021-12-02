import pyrealsense2 as rs
import matplotlib.pyplot as plt

from perception import RgbdSensorFactory
from camera_matrix import T_camera_world, T_robot_world
import numpy as np

def discover_cams():
    """Returns a list of the ids of all cameras connected via USB."""
    ctx = rs.context()
    ctx_devs = list(ctx.query_devices())
    ids = []
    for i in range(ctx.devices.size()):
        ids.append(ctx_devs[i].get_info(rs.camera_info.serial_number))
    return ids


def main(show_imgs=False):
    ids = discover_cams()
    assert ids, "[!] No camera detected."

    cfg = {}
    cfg["cam_id"] = ids[0]
    cfg["filter_depth"] = True
    cfg["frame"] = "camera"

    sensor = RgbdSensorFactory.sensor("realsense", cfg)
    sensor.start()
    camera_intr = sensor.color_intrinsics
    color_im, depth_im = sensor.frames()

    point_cloud_cam = camera_intr.deproject(depth_im)
    point_cloud_world = T_camera_world * point_cloud_cam
    point_cloud_robot = T_robot_world.inverse() * point_cloud_world

    if show_imgs:
        _, axes = plt.subplots(1, 2)
        for ax, im in zip(axes, [color_im.data, depth_im.data]):
            ax.imshow(im)
            ax.axis("off")
        plt.show()

    sensor.stop()

    return point_cloud_robot

if __name__ == "__main__":
    main(show_imgs=True)
