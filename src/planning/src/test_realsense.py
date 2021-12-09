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
    color_im, depth_im = sensor.frames()

    webcam_cfg = {}
    webcam_cfg["device_id"] = 3
    webcam_cfg["frame"] = "webcam"

    webcam_sensor = RgbdSensorFactory.sensor("webcam", webcam_cfg)
    webcam_sensor.start()
    webcam_im, _ = webcam_sensor.frames()

    depth_im = depth_im.inpaint(0.1)
    point_cloud_cam = camera_intr.deproject(depth_im)
    point_cloud_cam.remove_zero_points()
    point_cloud_cam.remove_infinite_points()

    min_pt = np.array([-0.4, -0.2, 0.4])
    max_pt = np.array([0.4, 0.2, 1])
    box = autolab_core.Box(min_pt, max_pt, frame=point_cloud_cam.frame)
    point_cloud_cam, _ = point_cloud_cam.box_mask(box)

    if show_imgs:
        v = pptk.viewer(point_cloud_cam.data.T)
        import pdb; pdb.set_trace()

    # point_cloud_cam = camera_intr.deproject_pixel(depth_im.data[250, 550], autolab_core.Point(np.array([250, 550]), frame=depth_im.frame))
    # point_cloud_cam.remove_zero_points()

    point_cloud_world = T_world_camera.inverse() * point_cloud_cam

    min_pt = np.array([-100, -0.4, 0.10])
    max_pt = np.array([100, 100, 0.15])
    box = autolab_core.Box(min_pt, max_pt, frame=point_cloud_world.frame)
    point_cloud_world_cup, _ = point_cloud_world.box_mask(box)

    if show_imgs:
        v = pptk.viewer(point_cloud_world_cup.data.T)
        import pdb; pdb.set_trace()

    kmeans = KMeans(n_clusters=2, random_state=0).fit(point_cloud_world_cup.data.T)
    labels = kmeans.labels_
    centers = kmeans.cluster_centers_

    if centers[0][1] < centers[1][1]:
        cup_left = point_cloud_world_cup.data[:, labels==0]
        center_left = centers[0:1, :]
        cup_right = point_cloud_world_cup.data[:, labels==1]
        center_right = centers[1]
    else:
        cup_left = point_cloud_world_cup.data[:, labels==1]
        center_left = centers[1:2, :]
        cup_right = point_cloud_world_cup.data[:, labels==0]
        center_right = centers[0]

    dists = np.linalg.norm(cup_left.T - center_left, axis=1)
    print(dists.shape)
    cup_left = cup_left[:, dists<0.1]

    dists = np.linalg.norm(cup_right.T - center_right, axis=1)
    print(dists.shape)
    cup_right = cup_right[:, dists<0.1]

    cup_left_top_pt_right = autolab_core.PointCloud(cup_left[:, np.argmax(cup_left[1, :])], frame=point_cloud_world.frame)
    cup_right_top_pt_right = autolab_core.PointCloud(cup_right[:, np.argmax(cup_right[1, :])], frame=point_cloud_world.frame)

    if show_imgs:
        v = pptk.viewer(point_cloud_world.data.T)
        v = pptk.viewer(cup_left.T)
        v = pptk.viewer(cup_right.T)
        import pdb; pdb.set_trace()

    point_cloud_robot = T_world_robot * point_cloud_world
    cup_left_top_pt_right_robot = T_world_robot * cup_left_top_pt_right
    cup_right_top_pt_right_robot = T_world_robot * cup_right_top_pt_right

    print(cup_left_top_pt_right_robot.data)
    print(cup_right_top_pt_right_robot.data)
    # import pdb; pdb.set_trace()

    if show_imgs:
        v = pptk.viewer(point_cloud_robot.data.T)
        import pdb; pdb.set_trace()

    if show_imgs:
        _, axes = plt.subplots(1, 2)
        for ax, im in zip(axes, [color_im.data, depth_im.data]):
            ax.imshow(im)
            ax.axis("off")
        plt.show()

    sensor.stop()
    webcam_sensor.stop()

    return point_cloud_robot, cup_left_top_pt_right_robot, cup_right_top_pt_right_robot, webcam_im

if __name__ == "__main__":
    get_robot_pointcloud(show_imgs=True)
