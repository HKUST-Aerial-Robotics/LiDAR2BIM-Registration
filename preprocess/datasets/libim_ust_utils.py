import os
import numpy as np
import open3d as o3d
from tqdm import tqdm
from copy import deepcopy
from preprocess.config import cfg
from preprocess.utils import mkdirs, clear_dirs
from preprocess.datasets.merge_pcds import PCDMerger
from preprocess.datasets.submap3d_manager import (
    Submap3dManager,
)
from preprocess.datasets.station_manager import StationManager
from preprocess.datasets.dir import DirManager

from preprocess.utils import (
    FileIO,
    xyyaw_to_se3,
)
from preprocess.geometry.bim import BIMManager
from preprocess.geometry.submap import SubmapManager
import matplotlib.pyplot as plt
class LiBIMUSTDataset:
    def __init__(self, cfg=None):
        self.session_trajecotries = None
        self.start_end_indices = []
        self.io = FileIO()
        if cfg is not None:
            self.load_cfg(cfg)

    def load_cfg(self, cfg):
        self.cfg = cfg
        self.seq = cfg.seq
        self.interval = cfg.interval
        self.dirs = DirManager(cfg)
        self.wall_pcds = []
        self.ground_pcds = []
        if self.seq != "Office":
            self.read_T_gtmap2bim()

    def load_bim_manager(self):
        self.bim = BIMManager(self.cfg)
        self.bim.load_bim(load_hash_table=False)

    def read_T_gtmap2bim(self):
        path = os.path.join(
            self.dirs.data_dir,
            "pose/pose_station_to_bim.txt",
        )
        if os.path.exists(path):
            self.T_gtmap2bim = np.genfromtxt(path).reshape((4, 4))
        else:
            raise FileNotFoundError(
                f"pose_station_to_bim not found, copy pose_station_to_bim.txt to the {path} "
            )

    def get_l2w_traj(self):
        # merge pcds preparation, get l2w, timestamps and trajectories
        traj_file = os.path.join(self.dirs.data_dir, "pose/odom.txt")
        pcd_dir = os.path.join(self.dirs.data_dir, "frame/points/")
        self.pcd_merger = PCDMerger(pcd_dir, traj_file)
        self.session_trajecotries = (
            self.T_gtmap2bim @ np.array(list(self.pcd_merger.gt_l2w.values()))
        )[:, :3, 3]

    def get_start2end(self, traj_len=None, next_start_len=None):
        if traj_len is None:
            traj_len = self.interval
        if next_start_len is None:
            next_start_len = self.cfg.next_start_len
        start_end_indices = []
        start_frame = 0
        start_frames = [start_frame]
        # get all the start frames, each start frame is 5m away from the previous start frame
        while start_frame < len(self.session_trajecotries):
            if self.cfg.split == "trajectory":
                start_frame = self.get_traj_end_frame(
                    start_frame, next_start_len, cfg.valid_len
                )
            elif self.cfg.split == "displacement":
                start_frame = self.get_disp_end_frame(
                    start_frame, next_start_len
                )
            if start_frame == -1:
                break
            start_frames.append(start_frame)
        # get the end frames
        for start_frame in start_frames:
            end_frame = -1
            if self.cfg.split == "trajectory":
                end_frame = self.get_traj_end_frame(
                    start_frame, traj_len, cfg.valid_len
                )
            elif self.cfg.split == "displacement":
                end_frame = self.get_disp_end_frame(
                    start_frame, traj_len
                )
            if end_frame != -1:
                start_end_indices.append((start_frame, end_frame))
        self.start_end_indices = start_end_indices
        if self.seq == "BuildingDay":
            indoor_frames = [[500, 1180], [1190, 2044], [2647, 3444], [4400, 5295]]
            self.purge_outdoor_frames(indoor_frames)

    def purge_outdoor_frames(self, indoor_frames):
        indoor_start_end_indices = []
        for se in self.start_end_indices:
            for indoor_frame in indoor_frames:
                if se[0] >= indoor_frame[0] and se[1] <= indoor_frame[1]:
                    indoor_start_end_indices.append(se)
                    break
        self.start_end_indices = indoor_start_end_indices

    def get_traj_end_frame(self, start_frame, traj_len, valid_len=2):
        # get the end frame of the trajectory, the distance between each pair of positions is larger than valid_len
        positions = self.session_trajecotries
        accumulated_dist = 0
        pt1 = start_frame
        for pt2 in range(start_frame, len(positions) - 1):
            dist = np.linalg.norm(positions[pt2] - positions[pt1])
            accumulated_dist_tmp = accumulated_dist + dist
            if dist > valid_len:
                accumulated_dist += dist
                pt1 = pt2
            if accumulated_dist_tmp > traj_len:
                return pt2
        return -1
    def get_disp_end_frame(self, start_frame, displacement):
        # the displacement is the distance between the start frame and the end frame
        positions = self.session_trajecotries
        # the endframe is the frame that is displacement away from the start frame
        end_frame = start_frame
        while end_frame < len(positions) - 1:
            dist = np.linalg.norm(positions[end_frame] - positions[start_frame])
            if dist > displacement:
                break
            end_frame += 1
        if end_frame == len(positions) - 1:
            return -1
        return end_frame

    def init_submap3d(self, deprecated=False):
        # submap3d is not aligned with gravity
        self.get_l2w_traj()
        self.get_start2end()
        station = StationManager(self.cfg)
        station_pcd = station.pcd

        self.submap3d_manager = Submap3dManager(
            self.pcd_merger,
            self.start_end_indices,
            station_pcd,
            self.cfg,
            deprecated=deprecated,
        )

    def generate_submap3d(self, viz=False, save=True, clear=True, deprecated=False):
        self.init_submap3d(deprecated=deprecated)
        if deprecated:
            self.submap3d_manager.make_benchmarks_deprecated(self.T_gtmap2bim)
        self.submap3d_manager.generate_submap3d(
            viz=viz, save=save, clear=clear, deprecated=deprecated
        )

    def generate_submap(self, index, start, end,mode='incremental', refine=True):
        submap = SubmapManager(index, self.cfg)

        if len(self.ground_pcds) > 0:
            submap.set_ground_pcds(self.ground_pcds)

        print(f"start: {start}, end: {end}")
        submap.generate_gt(self.T_gtmap2bim, self.bim.pcd, refine=refine)
        if mode == 'incremental':
            submap.generate_incremental_pcd2d(start, end, index)
            submap.generate_incremental_occupied_pcd(start, end)
            submap.generate_incremental_linesegs(start, end)
            imgpath = os.path.join(self.dirs.lineseg_dir, f"{index:06d}.png")
            plt.savefig(imgpath)
            submap.generate_free_pcd()

            self.ground_pcds = submap.ground_pcds
        elif mode == 'full':
            # return submap
            submap.generate_occupied_pcd()
            submap.generate_linsegs()
            imgpath = os.path.join(self.dirs.lineseg_dir, f"{index:06d}.png")
            plt.savefig(imgpath)
            submap.generate_free_pcd()
        return submap

    def make_benchmarks(self, viz=False, save=True, clear=True, refine=True):
        self.get_l2w_traj()
        self.get_start2end()
        mkdirs([self.dirs.seq_dir, self.dirs.occupied_pcd_flatten_dir, self.dirs.free_pcd_dir, self.dirs.lineseg_dir])
        if clear:
            clear_dirs([self.dirs.occupied_pcd_flatten_dir, self.dirs.free_pcd_dir, self.dirs.lineseg_dir])

        self.load_bim_manager()
        self.bim.load_bim(load_hash_table=False)
        xyyaw_list = []
        i = 0
        # self.start_end_indices = [(0, 100)]
        for (start, end) in tqdm(self.start_end_indices[:]):
            submap = self.generate_submap(i, start, end, refine=refine)
            xyyaw_list.append(submap.gt)
            if viz:
                T = xyyaw_to_se3(submap.gt[0], submap.gt[1], submap.gt[2], degrees=True)
                o3d_lineset = deepcopy(submap.lines.get_o3d_lineset()).transform(T)
                pcd = deepcopy(submap.occupied_pcd).transform(T)
                o3d.visualization.draw_geometries([pcd, o3d_lineset, self.bim.pcd])
            if save:
                occupied_pcd_save_path = os.path.join(self.dirs.occupied_pcd_dir, f"{i:06d}.pcd")
                occupied_pcd_flatten_save_path = os.path.join(self.dirs.occupied_pcd_flatten_dir, f"{i:06d}.pcd")
                free_pcd_save_path = os.path.join(self.dirs.free_pcd_dir, f"{i:06d}.pcd")
                lineseg_save_path = os.path.join(self.dirs.lineseg_dir, f"{i:06d}.txt")

                flatten_pcd = deepcopy(submap.occupied_pcd)
                points = np.asarray(flatten_pcd.points)
                points[:, 2] = 0
                flatten_pcd.points = o3d.utility.Vector3dVector(points)
                flatten_pcd = flatten_pcd.voxel_down_sample(0.05)

                # o3d.io.write_point_cloud(occupied_pcd_save_path, submap.occupied_pcd)
                o3d.io.write_point_cloud(occupied_pcd_flatten_save_path, flatten_pcd)
                o3d.io.write_point_cloud(free_pcd_save_path, submap.free_pcd)
                submap.lines.save_to_file(lineseg_save_path)

            i += 1

        gt_save_path = os.path.join(self.dirs.seq_dir, "pose.txt")
        if save:
            with open(gt_save_path, "w") as f:
                f.write("x y yaw\n")
                for xyyaw in xyyaw_list:
                    f.write(f"{xyyaw[0]} {xyyaw[1]} {xyyaw[2]}\n")
            print(f"pose.txt saved to {gt_save_path}.")

    def merge_office(self, remove_subset=True):

        # find all the office dir
        office_dirs = ["2f_office_01", "2f_office_02", "2f_office_03"]
        occupied_pcd_list = []
        free_pcd_list = []
        lineseg_list = []
        pose_list = []
        LiBIM_UST_ROOT = os.getenv("LiBIM_UST_ROOT")
        for office in office_dirs:
            subseq_dir = os.path.join(
                LiBIM_UST_ROOT,"processed", office, f"{self.interval:02d}m"
            )
            occupied_pcd_dir = os.path.join(subseq_dir, "occupied_pcd_flatten")
            free_pcd_dir = os.path.join(subseq_dir, "free_pcd")
            lineseg_dir = os.path.join(subseq_dir, "lineSeg")
            pose_path = os.path.join(subseq_dir, "pose.txt")

            occupied_pcd_list += self.io.pcd_from_dir(occupied_pcd_dir)
            free_pcd_list += self.io.pcd_from_dir(free_pcd_dir)
            lineseg_list += self.io.lineseg_from_dir(lineseg_dir)
            pose_list += self.io.pose_from_dir(pose_path)

        occupied_pcd_save_dir = self.dirs.occupied_pcd_flatten_dir
        free_pcd_save_dir = self.dirs.free_pcd_dir
        lineseg_save_dir = self.dirs.lineseg_dir
        pose_save_path = os.path.join(self.dirs.seq_dir, "pose.txt")

        mkdirs([occupied_pcd_save_dir,free_pcd_save_dir, lineseg_save_dir])
        clear_dirs([occupied_pcd_save_dir,free_pcd_save_dir, lineseg_save_dir])
        # save pcd
        for i, pcd in enumerate(occupied_pcd_list):
            o3d.io.write_point_cloud(os.path.join(occupied_pcd_save_dir, f"{i:06d}.pcd"), pcd)
        print(f"pcd saved to {occupied_pcd_save_dir}.")
        for i, pcd in enumerate(free_pcd_list):
            o3d.io.write_point_cloud(os.path.join(free_pcd_save_dir, f"{i:06d}.pcd"), pcd)
        print(f"pcd saved to {free_pcd_save_dir}.")
        # save lineseg
        for i, lineseg in enumerate(lineseg_list):
            with open(os.path.join(lineseg_save_dir, f"{i:06d}.txt"), "w") as f:
                f.writelines(lineseg)
        print(f"lineseg saved to {lineseg_save_dir}.")
        # save pose
        with open(pose_save_path, "w") as f:
            f.write("x y yaw\n")
            for pose in pose_list:
                f.write(pose)
        print(f"pose.txt saved to {pose_save_path}.")
        # if remove_subset:
        #     for office in office_dirs:
        #         subseq_dir = os.path.join(cfg.ROOT_DIR, "data/LiBIM-UST", office)
        #         # remove the subseq_dir
        #         os.system(f"rm -r {subseq_dir}")

    def check_pose(self, index):
        # check the pose of the start_frame and end_frame
        submap = SubmapManager(index, self.cfg)
        submap.load_submap()
        self.load_bim_manager()
        self.bim.load_bim(load_hash_table=False, load_lines=False)
        pcd = deepcopy(submap.occupied_pcd)
        o3d_lineset = deepcopy(submap.lines.get_o3d_lineset())
        T = xyyaw_to_se3(submap.gt[0], submap.gt[1], submap.gt[2], degrees=True)
        o3d_lineset = o3d_lineset.transform(T)
        pcd = pcd.transform(T)
        o3d.visualization.draw_geometries([pcd, self.bim.pcd, o3d_lineset])
        # o3d.visualization.draw_geometries([pcd, o3d_lineset, self.bim.pcd])

