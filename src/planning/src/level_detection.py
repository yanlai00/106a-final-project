import numpy as np
import cv2 
from autolab_core import ColorImage
import matplotlib.pyplot as plt

def onclick(self, event):
    self.ax.set_xlim(0, self.im_shape[1])
    self.ax.set_ylim(self.im_shape[0], 0)

    rc_coord = np.array([event.ydata, event.xdata])
    if (rc_coord[0] < self.restricted_space[0, 0]) or (rc_coord[0] > self.restricted_space[1, 0]) or \
        (rc_coord[1] < self.restricted_space[0, 1]) or (rc_coord[1] > self.restricted_space[1, 1]):
        print('outside restriced space!')
        return
    self.coords.append(rc_coord)


    if len(self.coords) == 3:
        new_grasp = np.stack(self.coords, axis=0)
        res = calc_grasp_corners(new_grasp)
        if res.grasp_width < MIN_GRASP_WIDTH or res.grasp_width > MAX_GRASP_WIDTH:
            print("grasp width {} out of range!".format(res.grasp_width))
            self.coords.pop(-1)
            return

        self.grasps.append(new_grasp)
        self.coords = []


def mask_edge(img, get_marker=True):
    # edges = cv2.Canny(image=img, threshold1=500, threshold2=600, apertureSize=7)
    edges = np.zeros(img.shape)
    if get_marker:
        mask_entries = np.where(np.all(img<100, axis=-1)) # coke 
    else:
        mask_entries = np.where(img[:, :, 2]<100) # code with water
        # mask_entries = np.where(np.any(img < 100, axis=-1)) # beads
        # mask_entries = np.where(img[:, :, 2]<130) # code with boba
    edges[mask_entries] = 1
    y_idx, x_idx = mask_entries
    if get_marker:
        bottom_pt = np.max(y_idx), np.mean(x_idx).astype(int)
    else: 
        # bottom_pt = np.min(y_idx), np.mean(x_idx).astype(int)
        bottom_pt = None
    return edges, bottom_pt

def preprocess_img(img, left, right, top, bottom):
    img = img[top:bottom, left:right, :]
    return cv2.GaussianBlur(img, (3, 3), sigmaX=0, sigmaY=0)

def return_gbr(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def show(img, point=None):
    if point is not None:
        y, x = point
        plt.plot(x, y, 'ro')
    try:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    except:
        img = img
    plt.imshow(img)
    plt.show()

def get_percentage(mask):
    return np.sum(mask) / np.prod(mask.shape)

def get_marker_edge(img, left, right, bottom, vis=False):
    img = preprocess_img(img, left, right, 0, bottom)
    if vis:
        show(img)
    edges, bottom_pt = mask_edge(img, get_marker=True)
    if vis:
        show(edges, bottom_pt)
        show(edges[bottom_pt[0]:,:])
    return bottom_pt[0] # y axis (axis 0) for edge

def get_percent_liquid(img, left, right, top, bottom, vis=False):
    img = preprocess_img(img, left, right, top, bottom)
    if vis:
        show(img)
    edges, bottom_pt = mask_edge(img, get_marker=False)
    if vis:
        show(edges, bottom_pt)
    percent = get_percentage(edges)
    print(percent)
    return percent

if __name__ == "__main__":
    # # preprocess empty cup
    # img_path = "/home/cc/ee106a/fl21/class/ee106a-aak/Desktop/2021-12-07-023713.jpg"
    # img = cv2.imread(img_path)
    # left, right = 630, 1180
    # bottom = 830
    # if left is None or right is None or bottom is None:
    #     show(img)
    #     left = int(raw_input("Left edge index"))
    #     right = int(raw_input("right edge index"))
    #     bottom = int(raw_input("bottom edge index"))
    # get_marker_edge(img, left, right, bottom, vis=True)

    # # post empty cup 
    img_path = "/home/cc/ee106a/fl21/class/ee106a-aak/Desktop/2021-12-16-211732.jpg"
    img = cv2.imread(img_path)
    left, right = 638, 1311
    top, bottom = 590, 886
    if left is None or right is None or bottom is None:
        show(img)
        left = int(raw_input("Left edge index"))
        right = int(raw_input("right edge index"))
        top = int(raw_input("top edge index"))
        bottom = int(raw_input("bottom edge index"))
    get_percent_liquid(img, left, right, top, bottom, vis=True)
