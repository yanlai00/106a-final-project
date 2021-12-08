import numpy as np
import cv2 
from autolab_core import ColorImage
import matplotlib.pyplot as plt

def mask_edge(img, get_marker=True):
    # edges = cv2.Canny(image=img, threshold1=500, threshold2=600, apertureSize=7)
    edges = np.zeros(img.shape)
    if get_marker:
        mask_entries = np.where(np.all(img<100, axis=-1)) # coke 
    else:
        mask_entries = np.where(img[:, :, 0] < 125)
    edges[mask_entries] = 1
    y_idx, x_idx = mask_entries
    if get_marker:
        bottom_pt = np.max(y_idx), np.mean(x_idx).astype(int)
    else: 
        bottom_pt = np.min(y_idx), np.mean(x_idx).astype(int)
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
    return get_percentage(edges)

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

    # post empty cup 
    # img_path = "/home/cc/ee106a/fl21/class/ee106a-aak/Desktop/2021-12-07-030232.jpg"
    img_path = "/home/cc/ee106a/fl21/class/ee106a-aak/Desktop/2021-12-08-040115.jpg"
    img = cv2.imread(img_path)
    left, right = 634, 1269
    top, bottom = 0, 1068
    if left is None or right is None or bottom is None:
        show(img)
        left = int(raw_input("Left edge index"))
        right = int(raw_input("right edge index"))
        top = int(raw_input("top edge index"))
        bottom = int(raw_input("bottom edge index"))
    get_percent_liquid(img, left, right, top, bottom, vis=True)
