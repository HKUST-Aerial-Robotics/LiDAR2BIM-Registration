import os
import numpy as np
import open3d as o3d
from preprocess.datasets.pcd2d_manager import PCD2dManager
from preprocess.datasets.dir import DirManager
from preprocess.geometry.point_cloud import (
    PointCloudManager,
)
from preprocess.geometry.lineseg import (
    LineSegmentExtractor,
    LineSegments,
    LineSegment,
    LineSegmentMerger,
)
from preprocess.config import load_cfg
from easydict import EasyDict
import pickle

import cv2
from preprocess.utils import setup_plot


def deprecated_save_o3d_lineset(o3d_lineset_path, o3d_lineset):
    points = np.asarray(o3d_lineset.points)
    lines = np.asarray(o3d_lineset.lines)

    lineset_data = {"points": points, "lines": lines}

    with open(o3d_lineset_path, "wb") as f:
        pickle.dump(lineset_data, f)


def export_for_deprecated(pcd2d_list, o3d_lineset, start_id, end_id, cfg):
    T = pcd2d_list[0].T_rp_inv
    o3d_lineset.transform(T)

    cfg.frontend_path = EasyDict()
    cfg.frontend_path.root = os.path.join(
        cfg.ROOT_DIR, "data/FusionBIM", "sequences", "bday", "cache"
    )
    o3d_lineset_path = os.path.join(
        cfg.frontend_path.root,
        "o3d_lineset",
        f"{start_id:04d}_{end_id:04d}_o3d_lineset.pkl",
    )
    deprecated_save_o3d_lineset(o3d_lineset_path, o3d_lineset)


class LineSegmentManager:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lineseg_merger = LineSegmentMerger(cfg)
        self.dir = DirManager(cfg)

    def viz_linesegs_on_img(self, img, linesegs):
        fig, ax = setup_plot(img.shape)
        ax.imshow(img, cmap="gray")
        for line in linesegs.linesegments:
            ax.plot(
                [line.point_a[0], line.point_b[0]],
                [line.point_a[1], line.point_b[1]],
                color="red",
                linewidth=2,
            )
        # plt.show()


    def frame_plane_based_extract(self, pcd2d_list):
        # this pcd2d is a frame
        self.lineseg_merger.extract(pcd2d_list)
        linesegments = self.lineseg_merger.merge(angle_thd=8, pt_thd=0.3)
        return linesegments

    def frame_image_based_extract(self, pcd2d_list):
        # this pcd2d is a frame
        walls_pcd = o3d.geometry.PointCloud()
        for pcd2d in pcd2d_list:
            walls_pcd += pcd2d.walls_2d
        walls_pcd.voxel_down_sample(voxel_size=0.1)
        # move min to origin
        T_min2og = np.eye(4)
        xy = np.min(np.array(walls_pcd.points), axis=0)
        T_min2og[0, 3] = -xy[0]
        T_min2og[1, 3] = -xy[1]
        walls_pcd.transform(T_min2og)

        # detect lines
        pc_manager = PointCloudManager(np.array(walls_pcd.points), self.cfg)
        pc_manager.get_img()
        img = np.array(pc_manager.png)
        img = 255 - img
        if self.cfg.seq == "BuildingDay":
            linesegs = hough_line_detection(img)
            linesegs.merge_uf(angle_thd=10, pt_thd=30)
            linesegs.cluster_angle()
            linesegs.remove_minor_angle(min_num=5, min_length=30)
        else:
            linesegs = hough_line_detection(img, threshold=60, minLineLength=30, maxLineGap=30)
            linesegs.merge_uf(angle_thd=10, pt_thd=10)
            linesegs.cluster_angle(bandwidth=0.01)
            linesegs.remove_minor_angle(min_num=5, min_length=60)
        self.viz_linesegs_on_img(img, linesegs)

        # move back
        rot1 = pc_manager.T_hw2xy[:2, :2]
        t1 = pc_manager.T_hw2xy[:2, 2]
        T_og2min = np.linalg.inv(T_min2og)
        rot2 = T_og2min[:2, :2]
        t2 = T_og2min[:2, 3]

        linesegs.transform(rot1, t1)
        linesegs.transform(rot2, t2)

        return linesegs

    def submap_plane_based_extract(self, pcd2d):
        # this pcd2d is a submap
        lse = LineSegmentExtractor(pcd2d.walls_2d)
        lse.extract_lineseg()
        linesegs = lse.linesegs
        return linesegs

    def submap_image_based_extract(self, pcd2d):
        # this pcd2d is a submap
        walls_pcd = pcd2d.walls_2d
        walls_pcd.voxel_down_sample(voxel_size=0.1)
        # move min to origin
        T_min2og = np.eye(4)
        xy = np.min(np.array(walls_pcd.points), axis=0)
        T_min2og[0, 3] = -xy[0]
        T_min2og[1, 3] = -xy[1]
        walls_pcd.transform(T_min2og)

        pc_manager = PointCloudManager(np.array(walls_pcd.points), self.cfg)
        pc_manager.get_img()
        img = np.array(pc_manager.png)
        img = 255 - img
        linesegs = hough_line_detection(img, threshold=50, minLineLength=30, maxLineGap=60)


        linesegs.merge_uf(angle_thd=10, pt_thd=10)
        # linesegs.cluster_angle()
        self.viz_linesegs_on_img(img, linesegs)

        # move back
        rot1 = pc_manager.T_hw2xy[:2, :2]
        t1 = pc_manager.T_hw2xy[:2, 2]
        T_og2min = np.linalg.inv(T_min2og)
        rot2 = T_og2min[:2, :2]
        t2 = T_og2min[:2, 3]

        linesegs.transform(rot1, t1)
        linesegs.transform(rot2, t2)

        return linesegs

    def generate_linesegs(self, pcd2d_manager, start_id, end_id, method="frame_plane"):
        linesegs = None
        if method == "frame_plane":
            pcd2d_list = pcd2d_manager.generate_pcd2d_list(
                num=end_id - start_id, start=start_id
            )
            linesegs = self.frame_plane_based_extract(pcd2d_list)
        elif method == "frame_image":
            pcd2d_list = pcd2d_manager.generate_pcd2d_list(
                num=end_id - start_id, start=start_id
            )
            linesegs = self.frame_image_based_extract(pcd2d_list)
        elif method == "submap_plane":
            pcd2d = pcd2d_manager.generate_pcd2d_list(num=1, start=start_id)[0]
            linesegs = self.submap_plane_based_extract(pcd2d)
        elif method == "submap_image":
            pcd2d = pcd2d_manager.generate_pcd2d_list(num=1, start=start_id)[0]
            linesegs = self.submap_image_based_extract(pcd2d)
        return linesegs


def hough_line_detection(img, threshold=80, minLineLength=30, maxLineGap=80):
    lines = cv2.HoughLinesP(
        img,
        1,
        np.pi / 180,
        threshold=threshold,
        minLineLength=minLineLength,
        maxLineGap=maxLineGap,
    )
    lineseg_list = []
    if lines is None:
        # return random linesegs
        for i in range(3):
            p1 = np.random.rand(2) * 100
            p2 = np.random.rand(2) * 100
            lineseg = LineSegment(p1, p2)
            lineseg_list.append(lineseg)
        linesegs = LineSegments(lineseg_list)
        return linesegs
    for line in lines:
        x1, y1, x2, y2 = line[0]
        p1 = np.array([x1, y1])
        p2 = np.array([x2, y2])
        lineseg = LineSegment(p1, p2)
        lineseg_list.append(lineseg)
    linesegs = LineSegments(lineseg_list)
    return linesegs


def lsd_line_detection(img):
    lsd = cv2.createLineSegmentDetector(0)
    lines = lsd.detect(img)[0]
    lineseg_list = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        p1 = np.array([x1, y1])
        p2 = np.array([x2, y2])
        lineseg = LineSegment(p1, p2)
        lineseg_list.append(lineseg)
    linesegs = LineSegments(lineseg_list)
    return linesegs


def contour_line_detection(img):
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    simplified_contours = []
    for cnt in contours:
        # 使用 approxPolyDP 近似轮廓
        epsilon = 0.05 * cv2.arcLength(cnt, True)  # epsilon 通常是周长的一小部分
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        simplified_contours.append(approx)

    # draw contours using matplotlib
    lineseg_list = []

    for contour in simplified_contours:
        contour = np.squeeze(contour, axis=1)
        for i in range(len(contour) - 1):
            p1 = contour[i]
            p2 = contour[i + 1]
            lineseg = LineSegment(p1, p2)
            lineseg_list.append(lineseg)
    linesegs = LineSegments(lineseg_list)
    return linesegs


if __name__ == "__main__":
    from preprocess.utils import FileIO
    config_path = (
        "/home/qzj/code/test/bim-relocalization/configs/interval/15m/building_day.yaml"
    )
    cfg = load_cfg(config_path)
    wall_pcd_dir = "/home/qzj/datasets/LiBIM-UST-raw/BuildingDay/frame/walls_points"
    ground_pcd_dir = (
        "/home/qzj/datasets/LiBIM-UST-raw/BuildingDay/submap3d/15m/ground_points"
    )

    dataset_path = os.path.join(
        cfg.ROOT_DIR,
        "data/FusionBIM",
        "sequences",
        "bday",
        "submap",
        "15.0m",
        "pcd",
    )
    io = FileIO()
    ground_pcds = io.pcd_from_dir(
        ground_pcd_dir, wall=False
    )
    name_list = []
    for item in sorted(os.listdir(dataset_path)):
        if "submap" not in item:
            continue
        name = item.split(".")[0]
        name_list.append(name)
    for index, name in enumerate(name_list):
        start_id = int(name.split("_")[1])
        end_id = int(name.split("_")[2])

        pcd2d_manager = PCD2dManager(wall_pcd_dir, ground_pcd_dir, cfg)


        pcd2d_list = pcd2d_manager.generate_pcd2d_list_v2(index, num=end_id - start_id, start=start_id, ground_pcds=ground_pcds)

        ls_manager = LineSegmentManager(cfg)

        linesegments = ls_manager.frame_plane_based_extract(pcd2d_list)
        o3d_lineset = linesegments.get_o3d_lineset()
        # o3d_linset_list = ls_manager.get_o3d_lineset()
        # o3d.visualization.draw_geometries([o3d_lineset])
        export_for_deprecated(pcd2d_list, o3d_lineset, start_id, end_id,cfg)
        # ref = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #     size=55, origin=[0, 0, 0]
        # )
        # T = np.eye(4)
        # T[2, 3] = -15
        # o3d_linset_list = [o3d_l.transform(T) for o3d_l in o3d_linset_list]
        # T2 = np.eye(4)
        # T2[0, 3] = 100
        # walls_pcd = [pcd2d.og_walls_cropped for pcd2d in pcd2d_manager.pcd2d_list]
        # o3d.visualization.draw_geometries(
        #     [o3d_lineset, deepcopy(o3d_lineset).transform(T2), ref]
        #     + walls_pcd
        #     + o3d_linset_list
        # )