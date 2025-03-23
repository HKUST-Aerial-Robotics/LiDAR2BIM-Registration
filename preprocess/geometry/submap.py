import os

import numpy as np
import open3d as o3d
from preprocess.geometry.point_cloud import PointCloudManager
from preprocess.datasets.dir import DirManager
from preprocess.datasets.pcd2d_manager import PCD2dManager
from preprocess.datasets.lineseg_manager import LineSegmentManager
from preprocess.geometry.bim import BaseMapManager
from preprocess.utils import se32xyyaw, icp_registration
from copy import deepcopy


class SubmapManager(BaseMapManager):
    def __init__(self, index, cfg):
        super(SubmapManager, self).__init__(cfg)
        self.index = index

        self.dir = DirManager(cfg)
        self.seq_dir = self.dir.seq_dir

        self.pc_manager = None
        self.pcd2d_manager = None
        self.lineseg_manager = None
        self.gt = None

        self.wall_pcds = []
        self.ground_pcds = []
    def clear(self):
        self.occupied_pcd = None
        self.free_pcd = None
        self.lines = None
        self.pc_manager = None
        self.pcd2d_manager = None
        self.lineseg_manager = None
        self.gt = None
        self.incremental_pcd2d_list = None
    def load_submap(self):

        occupied_pcd_file = os.path.join(self.seq_dir, f"occupied_pcd/{self.index:06d}.pcd")
        free_pcd_file = os.path.join(self.seq_dir, f"free_pcd/{self.index:06d}.pcd")
        self.occupied_pcd = self.load_pcd(occupied_pcd_file)
        self.free_pcd = self.load_pcd(free_pcd_file, color=(0, 0.651, 0.929))

        self.pc_manager = PointCloudManager(np.array(self.occupied_pcd.points), self.cfg)

        linesegs_file = os.path.join(self.seq_dir, f"lineSeg/{self.index:06d}.txt")
        self.lines = self.load_lines(linesegs_file)

        self.gt = self.load_gt(os.path.join(self.seq_dir, "pose.txt"))

        return self.occupied_pcd, self.lines, self.hash_table

    def load_pcd(self, path, color=(0, 0.651, 0.929)):
        # reading bim_pcd
        if os.path.exists(path):
            pcd = o3d.io.read_point_cloud(path)
        else:
            raise FileNotFoundError(
                f"submap_pcd not found, copy bim.pcd to the {path} "
            )
        pcd.paint_uniform_color(color)
        return pcd

    def load_gt(self, path):
        if os.path.exists(path):
            # gt file is a txt file with 3 values, x, y, yaw for each line
            with open(path, "r") as f:
                gt_list = f.readlines()
                # the first line is the header
                gt_list = [list(map(float, x.strip().split())) for x in gt_list[1:]]
            gt = gt_list[self.index]
        else:
            raise FileNotFoundError(f"gt not found, copy gt to the {path} ")
        return gt

    def filter_ground(self, free_pcd):
        pcd = o3d.geometry.PointCloud()
        points = np.array(free_pcd.points)
        points = points[points[:, 2] < 0.01]
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd
    def set_wall_pcds(self, wall_pcds):
        self.wall_pcds = wall_pcds
    def set_ground_pcds(self, ground_pcds):
        self.ground_pcds = ground_pcds
    def generate_incremental_pcd2d(self, start, end, index=None):
        # self.pcd2d_manager = PCD2dManager(
        #     self.dir.wall_pcd_dir, self.dir.ground_pcd_dir, self.cfg
        # )
        # pcd2d_list = self.pcd2d_manager.generate_pcd2d_list(
        #     num=end - start, start=start
        # )

        self.pcd2d_manager = PCD2dManager(
            self.dir.wall_pcd_dir, self.dir.submap3d_ground_pcd_dir, self.cfg
        )

        if len(self.ground_pcds) > 0:
            self.pcd2d_manager.set_ground_pcds(self.ground_pcds)
        pcd2d_list = self.pcd2d_manager.generate_pcd2d_list_v2(index=index,
            num=end - start, start=start,
        )
        self.incremental_pcd2d_list = pcd2d_list
        self.wall_pcds = self.pcd2d_manager.wall_pcds
        self.ground_pcds = self.pcd2d_manager.ground_pcds
        # return pcd2d_list
    def get_T_rp(self):
        self.pcd2d_manager = PCD2dManager(
            self.dir.submap3d_wall_pcd_dir, self.dir.submap3d_ground_pcd_dir, self.cfg
        )
        pcd2d = self.pcd2d_manager.generate_pcd2d_list(num=1, start=self.index)[0]
        return pcd2d.T_rp
    def generate_pcd2d(self):
        self.pcd2d_manager = PCD2dManager(
            self.dir.submap3d_wall_pcd_dir, self.dir.submap3d_ground_pcd_dir, self.cfg
        )
        pcd2d = self.pcd2d_manager.generate_pcd2d_list(num=1, start=self.index)[0]
        return pcd2d

    def generate_incremental_occupied_pcd(self, start, end):
        if self.incremental_pcd2d_list is None:
            self.generate_incremental_pcd2d(start, end)
        walls_pcd = o3d.geometry.PointCloud()
        for pcd2d in self.incremental_pcd2d_list:
            walls_pcd += pcd2d.og_walls_cropped
        self.occupied_pcd = walls_pcd.voxel_down_sample(0.01)
        return walls_pcd
    def generate_incremental_free_pcd(self, start, end):
        if self.incremental_pcd2d_list is None:
            self.generate_incremental_pcd2d(start, end)
        free_pcd = o3d.geometry.PointCloud()
        for pcd2d in self.incremental_pcd2d_list:
            free_pcd += pcd2d.og_ground
        free_pcd = free_pcd.voxel_down_sample(voxel_size=0.3)
        free_pcd = self.filter_ground(free_pcd)
        self.free_pcd = free_pcd
        return free_pcd
    def generate_occupied_pcd(self):
        pcd2d = self.generate_pcd2d()
        self.occupied_pcd = pcd2d.og_walls_cropped
        return self.occupied_pcd
    def generate_free_pcd(self):
        pcd2d = self.generate_pcd2d()
        self.free_pcd = pcd2d.og_ground
        self.free_pcd = self.free_pcd.voxel_down_sample(voxel_size=0.3)
        self.free_pcd = self.filter_ground(self.free_pcd)
        return self.free_pcd

    def generate_incremental_linesegs(self, start, end):
        if self.incremental_pcd2d_list is None:
            self.generate_incremental_pcd2d(start, end)
        self.lineseg_manager = LineSegmentManager(self.cfg)
        linesegs = self.lineseg_manager.frame_image_based_extract(self.incremental_pcd2d_list)

        self.lines = linesegs
        o3d_lineset = linesegs.get_o3d_lineset()
        # o3d.visualization.draw_geometries([o3d_lineset, self.occupied_pcd])
        return linesegs

    def generate_linsegs(self):
        pcd2d = self.generate_pcd2d()
        self.lineseg_manager = LineSegmentManager(self.cfg)
        linesegs = self.lineseg_manager.submap_image_based_extract(pcd2d)

        self.lines = linesegs
        # o3d_lineset = linesegs.get_o3d_lineset()
        # o3d.visualization.draw_geometries([o3d_lineset, self.pcd])
        return linesegs

    def generate_gt(self, T_gtmap2bim, bim_pcd, refine=True):
        pcd2d = self.generate_pcd2d()
        gt_pose = T_gtmap2bim @ np.linalg.inv(pcd2d.T_rp)
        if refine:
            # use icp to refine
            source = np.array(pcd2d.og_walls.points)
            target = np.array(bim_pcd.points)
            gt_pose = deepcopy(icp_registration(source, target, gt_pose))
        xyyaw = se32xyyaw(gt_pose, degrees=True)
        self.gt = xyyaw
        return xyyaw
