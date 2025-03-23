from preprocess.utils import FileIO
from preprocess.utils import mkdirs, clear_dirs
from preprocess.geometry.point_cloud import PCD2d
import open3d as o3d
import os


class PCD2dManager:
    def __init__(self, walls_pcd_dir, ground_pcd_dir, cfg):
        self.io = FileIO()
        self.walls_pcd_dir = walls_pcd_dir
        self.ground_pcd_dir = ground_pcd_dir
        self.wall_pcds = []
        self.ground_pcds = []
        self.cfg = cfg
        self.pcd2d_list = []

    def generate_pcd2d_list(self, num=-1, start=0, viz=False, lidar_origin="default"):
        # print(
        #     "Generating PCD2d..., reading pcds from",
        #     self.walls_pcd_dir,
        #     self.ground_pcd_dir,
        # )
        wall_pcds = self.io.pcd_from_dir(
            self.walls_pcd_dir, num=num, start=start, wall=True
        )
        ground_pcds = self.io.pcd_from_dir(
            self.ground_pcd_dir, num=num, start=start, wall=False
        )
        pcd2d_list = []
        num = 0
        for walls_pcd, ground_pcd in zip(wall_pcds, ground_pcds):
            pcd2d = PCD2d(walls_pcd, ground_pcd, self.cfg)
            if pcd2d.get_pcd2d(lidar_origin=lidar_origin):
                pcd2d_list.append(pcd2d)

            if viz:
                o3d.visualization.draw_geometries([pcd2d.og_walls_cropped])
            num += 1
        # self.pcd2d_list = pcd2d_list
        return pcd2d_list

    def set_wall_pcds(self, wall_pcds):
        self.wall_pcds = wall_pcds
    def set_ground_pcds(self, ground_pcds):
        self.ground_pcds = ground_pcds
    def generate_pcd2d_list_v2(self,index, num=-1, start=0, viz=False, lidar_origin="default", ground_pcds=None):
        if len(self.wall_pcds) == 0:
            self.wall_pcds = self.io.pcd_from_dir(
                self.walls_pcd_dir, num=num, start=start, wall=True
            )
        if len(self.ground_pcds) == 0:
            if ground_pcds is None:
                self.ground_pcds = self.io.pcd_from_dir(
                    self.ground_pcd_dir, wall=False,progress=True
                )
            else:
                self.ground_pcds = ground_pcds
        # print(
        #     "Generating PCD2d..., reading pcds from",
        #     self.walls_pcd_dir,
        #     self.ground_pcd_dir,
        # )
        pcd2d_list = []
        num = 0
        for walls_pcd in self.wall_pcds:
            pcd2d = PCD2d(walls_pcd, self.ground_pcds[index].voxel_down_sample(0.3), self.cfg)
            if pcd2d.get_pcd2d(lidar_origin=lidar_origin):
                pcd2d_list.append(pcd2d)
            if viz:
                o3d.visualization.draw_geometries([pcd2d.og_walls_cropped])
            num += 1
        # self.pcd2d_list = pcd2d_list
        return pcd2d_list
    def save_pcd2d(self, save_dir):
        mkdirs(save_dir)
        clear_dirs(save_dir)
        num = 0
        for pcd2d in self.pcd2d_list:
            o3d.io.write_point_cloud(
                os.path.join(
                    save_dir,
                    f"{num:06d}.pcd",
                ),
                pcd2d.og_walls_cropped,
            )
            print(f"{num:06d}.pcd saved to {save_dir}.")
            num += 1
