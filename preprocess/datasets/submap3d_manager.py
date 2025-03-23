import os
import numpy as np
import open3d as o3d
from tqdm import tqdm
from preprocess.utils import mkdirs, clear_dirs
from preprocess.datasets.dir import DirManager


class Submap3dManager:  # submap3d is not aligned with gravity
    def __init__(
        self, pcd_merger, start_end_indices, station_pcd, cfg, deprecated=False
    ):
        self.pcd_merger = pcd_merger
        self.start_end_indices = start_end_indices
        self.station_pcd = station_pcd
        self.cfg = cfg
        self.dirs = DirManager(cfg)

        self.submap3d_dir = self.dirs.submap3d_dir
        self.submap3d_wall_pcd_dir = self.dirs.submap3d_wall_pcd_dir
        self.submap3d_ground_pcd_dir = self.dirs.submap3d_ground_pcd_dir
        if deprecated:
            self.submap3d_dir = self.dirs.submap3d_deprecated_dir
            self.submap3d_wall_pcd_dir = self.dirs.submap3d_wall_pcd_deprecated_dir
            self.submap3d_ground_pcd_dir = self.dirs.submap3d_ground_pcd_deprecated_dir

    def generate_submap3d(self, viz=False, save=True, clear=True, deprecated=False):
        save_dir = self.submap3d_dir
        wall_pcd_dir = self.submap3d_wall_pcd_dir
        ground_pcd_dir = self.submap3d_ground_pcd_dir

        mkdirs([save_dir, wall_pcd_dir, ground_pcd_dir])
        if clear:
            clear_dirs([save_dir, wall_pcd_dir, ground_pcd_dir])

        num = 0
        for i, j in tqdm(self.start_end_indices):
            # print(f"Generating submap_{i:04d}_{j:04d}...")
            submap3d_pcd = self.accumulate_points(i, j, self.station_pcd,mode="pointlio", viz=viz)
            if save:
                if deprecated:
                    o3d.io.write_point_cloud(
                        os.path.join(save_dir, f"submap_{i:04d}_{j:04d}.pcd"),
                        submap3d_pcd,
                    )
                    print(f"submap_{i:04d}_{j:04d}.pcd saved to {save_dir}.")
                else:
                    o3d.io.write_point_cloud(
                        os.path.join(save_dir, f"{num:06d}.pcd"), submap3d_pcd
                    )
                    # print(f"{num:06d}.pcd saved to {save_dir}.")
            num += 1

    def make_benchmarks_deprecated(self, T_gtmap2bim):
        save_dir = os.path.join(os.path.dirname(self.submap3d_dir), "gt")
        mkdirs(save_dir)
        save_path = os.path.join(
            save_dir,
            "test_{}.txt".format(self.cfg.seq),
        )
        with open(save_path, "w") as f:
            f.write(
                "seq i j srctgt mot1 mot2 mot3 mot4 mot5 mot6 mot7 mot8 mot9 mot10 mot11 mot12 mot13 mot14 mot15\n"
            )

            for i, j in tqdm(self.start_end_indices):
                srctgt = "submap_" + "bim"
                f.write("%s %d %d %s" % (self.cfg.seq, i, j, srctgt))
                for k in range(len(T_gtmap2bim.flatten())):
                    f.write(" %f" % T_gtmap2bim.flatten()[k])
                f.write("\n")

    def accumulate_points(self, i, j, station_pcd,mode="pointlio", viz=False):
        if mode == "pointlio":
            pcd = self.pcd_merger.merge_pcd_pointlio(i, j)
        elif mode == "metacam":
            pcd = self.pcd_merger.merge_pcd(i, j, station_pcd)
        elif mode == "SLAM_S10":
            pcd = self.pcd_merger.merge_pcd_SLAMS10(i, j, station_pcd)
        pcd = pcd.voxel_down_sample(voxel_size=self.cfg.voxel)
        if viz:
            o3d.visualization.draw_geometries([pcd])
        return pcd

    def visualize_submap_pcd(self):
        submap_pcds = []
        for submap_pcd in os.listdir(self.submap3d_dir):
            # each submap_pcd has a unique color
            color = np.random.rand(3)
            # to ensure the color is very different from the previous one
            while (
                len(submap_pcds) > 0
                and np.linalg.norm(color - submap_pcds[-1].colors) < 0.8
            ):
                color = np.random.rand(3)

            submap_pcd = o3d.io.read_point_cloud(
                os.path.join(self.submap3d_dir, submap_pcd)
            )
            submap_pcd.paint_uniform_color(color)
            submap_pcds.append(submap_pcd)
        o3d.visualization.draw_geometries(submap_pcds)

    def calculate_submap_size(self):
        submap_pcds = []
        for submap_pcd in os.listdir(self.submap3d_dir):
            submap_pcd = o3d.io.read_point_cloud(
                os.path.join(self.submap3d_dir, submap_pcd)
            )
            submap_pcds.append(submap_pcd)
            print(f"submap_{submap_pcd}")
        # average number of points in each submap
        average_point_num = sum(
            [len(submap_pcd.points) for submap_pcd in submap_pcds]
        ) / len(submap_pcds)
        print(f"average_point_num: {average_point_num}")
        # standard deviation of the number of points in each submap
        std_point_num = np.std([len(submap_pcd.points) for submap_pcd in submap_pcds])
        print(f"std_point_num: {std_point_num}")
        return average_point_num, std_point_num


class KeyFrameMap3dManager(Submap3dManager):
    def __init__(
        self, pcd_merger, start_end_indices, station_pcd, cfg, deprecated=False
    ):
        super().__init__(pcd_merger, start_end_indices, station_pcd, cfg, deprecated)
        self.submap3d_dir = self.dirs.kfmap3d_dir
        self.submap3d_wall_pcd_dir = self.dirs.kfmap3d_wall_pcd_dir
        self.submap3d_ground_pcd_dir = self.dirs.kfmap3d_ground_pcd_dir

