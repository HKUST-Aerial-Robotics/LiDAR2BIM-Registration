import numpy as np
from preprocess.geometry.lineseg import LineSegment
from preprocess.geometry.corner import Corner
from preprocess.geometry.pc2img import Points2Image
from preprocess.utils import stats
from itertools import combinations

import open3d as o3d
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from typing import List
import gc

class PointCloudManager:
    def __init__(self, points: np.ndarray, cfg):
        self.cfg = cfg
        self.points = points

        self.png = None
        self.T_xyz2hw = None
        self.T_hw2xyz = None
        self.T_hw2xy = None

    def extract_hash_descriptors(self, corners: List[Corner]):
        linesegments = []
        if self.cfg.desc_name == "triangle":
            pass
        elif self.cfg.desc_name == "linesegment":
            linesegments = [
                LineSegment().from_corners(corners[i], corners[j])
                for i, j in combinations(range(len(corners)), 2)
            ]
        elif self.cfg.desc_name == "TriDescV2":
            pass
        elif self.cfg.desc_name == "TriDescV3":
            pass
        elif self.cfg.desc_name == "TriDescGr6D":
            pass
        elif self.cfg.desc_name == "TriDescGr7D":
            pass
        elif self.cfg.desc_name == "TriDescGr9D":
            pass
        stats.add("corners", len(corners))
        stats.add("linesegments", len(linesegments))
        geom_sets = self.desc_hashtable.build_geom_set(corners, linesegments, radius=self.cfg.max_side_length)
        stats.add("geom_sets", len(geom_sets))
        # Build hash table
        self.desc_hashtable.build_hash_table(geom_sets)
        return True

    def get_img(self):
        pc2img = Points2Image(self.points, self.cfg)
        self.png = pc2img.get_png()
        self.T_xyz2hw = pc2img.T_xyz2hw
        self.T_hw2xyz = pc2img.T_hw2xyz
        self.T_hw2xy = pc2img.T_hw2xy


class PCD2d:
    def __init__(
        self,
        walls_pcd: o3d.geometry.PointCloud,
        ground_pcd: o3d.geometry.PointCloud,
        cfg,
    ):
        self.walls_pcd = walls_pcd
        self.ground_pcd = ground_pcd
        self.cfg = cfg
        self.ref = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=55, origin=[0, 0, 0]
        )
        self.upper_bound = cfg.pcd2d.max_height
        self.lower_bound = cfg.pcd2d.min_height
        self.voxel_size = cfg.voxel

        self.T_rp = np.eye(4)
        self.T_rp_inv = np.eye(4)
        self.og_walls = None
        self.og_ground = None
        self.og_walls_cropped = None
        self.walls_2d = None

    # @profile
    def get_pcd2d(self, lidar_origin="default"):
        # reset rp to 0 to align gravity
        if not self.reset_rp(lidar_origin=lidar_origin) or len(self.walls_pcd.points) == 0:  # rpy and translation
            return False
        # crop the walls
        self.crop_walls()
        # self.viz_crop_walls()

        # flatten the walls
        self.flatten_walls()
        return True

    def clear_cache(self):
        refs = gc.get_referrers(self.ground_pcd)
        print(f"Number of referrers: {len(refs)}")

        # 输出每个引用对象的信息
        for i, ref in enumerate(refs):
            print(f"Referrer {i}: {ref}")
            print(f"Type: {type(ref)}")
            if isinstance(ref, dict):
                print(f"Keys: {list(ref.keys())[:10]}")  # 只打印前10个键
            elif isinstance(ref, list):
                print(f"First 10 elements: {ref[:10]}")  # 只打印前10个元素
            elif isinstance(ref, tuple):
                print(f"First 10 elements: {ref[:10]}")  # 只打印前10个元素
            print('-' * 80)

        del self.og_walls
        del self.og_ground
        del self.og_walls_cropped
        del self.walls_2d
        del self.walls_pcd
        del self.ground_pcd

        gc.collect()

    def reset_rp(self, lidar_origin="default") -> np.ndarray:
        # timer.tic("fit_plane")
        down_ground = self.ground_pcd.voxel_down_sample(voxel_size=1.0)
        if len(down_ground.points) < 3:
            print("No enough points to fit the plane.")
            return False
        # plane = fit_plane_PCA(np.array(down_ground.points))
        # timer.toc("fit_plane")
        plane = down_ground.segment_plane(0.1, 3, 20)[0]
        t_l2w = np.array([0, 0, 0])  # lidar to world
        if lidar_origin == "default":
            t_l2w = proj_pt2pl(t_l2w, plane)
        if lidar_origin == "min":
            wall_min = np.min(np.array(self.walls_pcd.points), axis=0)
            wm_on_plane = proj_pt2pl(wall_min, plane)
            t_l2w = wm_on_plane

        v1 = np.array([0, 0, 1])
        v2 = np.array(plane[:3])
        R = R_from_2vecs(v1, v2)
        t = -np.dot(R.T, t_l2w)
        T = np.eye(4)
        T[:3, :3] = R.T
        T[:3, 3] = t

        self.T_rp = T
        self.T_rp_inv = np.linalg.inv(T)
        self.og_walls = deepcopy(self.walls_pcd).transform(self.T_rp)
        self.og_ground = deepcopy(self.ground_pcd).transform(self.T_rp)
        return True

    def crop_walls(self):
        # get the crop box
        bound_pcd = get_bound_pcd(self.og_walls)
        crop_box = get_crop_box(bound_pcd, self.upper_bound, self.lower_bound)
        # crop the walls
        self.og_walls_cropped = self.og_walls.crop(crop_box)
    def viz_crop_walls(self):
        o3d.visualization.draw_geometries(
            [
                self.og_ground,
                deepcopy(self.og_walls_cropped).paint_uniform_color([1, 0, 0]),
                self.ref,
            ]
        )

    def flatten_walls(self):
        # set wall z to 0
        xyz = np.array(self.og_walls_cropped.points)
        xyz[:, 2] = 0
        walls_2d = o3d.geometry.PointCloud()
        walls_2d.points = o3d.utility.Vector3dVector(xyz)
        walls_2d.colors = self.og_walls_cropped.colors
        walls_2d = walls_2d.voxel_down_sample(voxel_size=self.voxel_size)
        self.walls_2d = walls_2d



def R_from_2vecs(v1, v2) -> np.ndarray:
    # v1, v2: 3x1 np.array
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = np.cross(v1, v2)
    axis_length = np.linalg.norm(axis)
    angle = np.arccos(np.dot(v1, v2))
    return R.from_rotvec(axis / axis_length * angle).as_matrix()


def fit_plane_PCA(points):
    centroid = np.mean(points, axis=0)
    points_centered = points - centroid

    H = np.dot(points_centered.T, points_centered)

    eigenvalues, eigenvectors = np.linalg.eigh(H)

    normal = eigenvectors[:, np.argmin(eigenvalues)]
    d = -np.dot(normal, centroid)
    plane = np.hstack([normal, d])

    return plane


def proj_pt2line(pt, mean_pt, v):
    t = np.dot(pt - mean_pt, v) / np.dot(v, v)
    return mean_pt + t * v


def get_pt2pl_dist(pt: np.ndarray, plane) -> float:
    a, b, c, d = plane
    return a * pt[0] + b * pt[1] + c * pt[2] + d / np.sqrt(a**2 + b**2 + c**2)


def proj_pt2pl(pt: np.ndarray, plane) -> np.ndarray:
    a, b, c, d = plane
    x, y, z = pt
    pt2pl_dist = get_pt2pl_dist(pt, plane)
    norm = np.sqrt(a**2 + b**2 + c**2)
    return np.array(
        [
            x - a * pt2pl_dist / norm,
            y - b * pt2pl_dist / norm,
            z - c * pt2pl_dist / norm,
        ]
    )


def get_crop_box(bound_pcd, upper_bound, lower_bound):
    bound_pts = np.array(bound_pcd.points)
    # set z to 0
    bound_pts[:, 2] = 0
    normal = np.array([0, 0, 1])

    # get the min and max of the bound_pts_on_plane
    min_bound, max_bound = get_min_max(bound_pts)
    min_bound = min_bound + normal * lower_bound
    max_bound = max_bound + normal * upper_bound

    crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    crop_box.color = [0, 1, 0]
    return crop_box


def get_bound_pcd(pcd) -> o3d.geometry.PointCloud:
    aabb = pcd.get_axis_aligned_bounding_box()
    min_bound = aabb.get_min_bound()
    max_bound = aabb.get_max_bound()
    bound_pcd = o3d.geometry.PointCloud()
    bound_pcd.points = o3d.utility.Vector3dVector(
        np.array(
            [
                min_bound,
                [max_bound[0], min_bound[1], min_bound[2]],
                [max_bound[0], max_bound[1], min_bound[2]],
                [min_bound[0], max_bound[1], min_bound[2]],
                min_bound,
            ]
        )
    )
    return bound_pcd


def get_min_max(bound_pts: np.ndarray) -> tuple:
    min_x = np.min(bound_pts[:, 0])
    max_x = np.max(bound_pts[:, 0])
    min_y = np.min(bound_pts[:, 1])
    max_y = np.max(bound_pts[:, 1])

    # the point should be on the plane
    min_z = 0
    max_z = 0

    min_bound = np.array([min_x, min_y, min_z])
    max_bound = np.array([max_x, max_y, max_z])

    return min_bound, max_bound
