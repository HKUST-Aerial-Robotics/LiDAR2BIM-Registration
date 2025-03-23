from PIL import Image
import numpy as np
import cv2
from preprocess.utils import setup_plot
import matplotlib.pyplot as plt

class Points2Image:
    def __init__(self, points: np.ndarray, cfg):
        self.cfg = cfg
        self.scale = cfg.pcd2d.scale
        self.dilate_kernel = cfg.pcd2d.dilate_kernel

        self.walls_pts = points
        self.T_xyz2hw = None
        self.T_hw2xyz = None

    def get_hw_mat(self, hw: np.ndarray, img_size) -> np.ndarray:
        H, W = img_size
        hw = np.round(hw).astype(int)
        hw = hw[(hw[:, 0] >= 0) & (hw[:, 0] < W) & (hw[:, 1] >= 0) & (hw[:, 1] < H)]
        hw_mat = np.zeros(img_size, dtype=np.uint8)
        hw_mat[hw[:, 1], hw[:, 0]] = 255
        return hw_mat

    def get_png(self) -> Image:
        xyz = self.walls_pts
        # set z to 0
        xyz[:, 2] = 0
        # xyz2hw_mat
        W, H = np.max(xyz, axis=0)[:2].astype(int) * self.scale
        img_size = (H + 1, W + 1)

        self.T_xyz2hw = xyz2hw(self.scale, img_size)
        self.T_hw2xyz = hw2xyz(self.scale, img_size)
        self.T_hw2xy = hw2xy(self.scale, img_size)

        xyz1 = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
        hw = np.dot(self.T_xyz2hw, xyz1.T).T[:, :2]
        hw_mat = self.get_hw_mat(hw, img_size)

        return Image.fromarray(255 - dilate(hw_mat, self.dilate_kernel))
    def viz(self, img=None):
        if img is None:
            img = self.get_png()
        img = np.array(img)
        fig, ax = setup_plot(img.shape)
        ax.imshow(img, cmap="gray")
        plt.show()

def xyz2hw(scale, img_size) -> np.ndarray:
    H, W = img_size
    T = np.array([[scale, 0, 0, 0], [0, -scale, 0, H], [0, 0, 0, 1]])
    return T


def hw2xyz(scale, img_size) -> np.ndarray:
    H, W = img_size
    T = np.array([[1 / scale, 0, 0], [0, -1 / scale, H / scale], [0, 0, 0], [0, 0, 1]])
    return T


def hw2xy(scale, img_size) -> np.ndarray:
    H, W = img_size
    T = np.array([[1 / scale, 0, 0], [0, -1 / scale, H / scale], [0, 0, 1]])
    return T


def dilate(np_img: np.ndarray, dilate_kernel: int) -> np.ndarray:
    kernel = np.ones((dilate_kernel, dilate_kernel), np.uint8)
    return cv2.dilate(np_img, kernel, iterations=1)
