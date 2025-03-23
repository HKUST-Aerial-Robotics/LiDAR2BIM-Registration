import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import open3d as o3d
import os
import numba as nb
from typing import Dict
import matplotlib.pyplot as plt
import math
from preprocess.config import cfg
from easydict import EasyDict


@nb.jit(nopython=True)
def curl_velodyne_data(velo_in: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    pt_num = velo_in.shape[0]
    velo_out = np.zeros_like(velo_in)

    for i in range(pt_num):
        vx, vy, vz = velo_in[i, :3]

        s = 0.5 * np.arctan2(vy, vx) / np.pi

        rx, ry, rz = s * r
        tx, ty, tz = s * t

        theta = np.sqrt(rx * rx + ry * ry + rz * rz)

        if theta > 1e-10:
            kx, ky, kz = rx / theta, ry / theta, rz / theta

            ct = np.cos(theta)
            st = np.sin(theta)

            kv = kx * vx + ky * vy + kz * vz

            velo_out[i, 0] = (
                vx * ct + (ky * vz - kz * vy) * st + kx * kv * (1 - ct) + tx
            )
            velo_out[i, 1] = (
                vy * ct + (kz * vx - kx * vz) * st + ky * kv * (1 - ct) + ty
            )
            velo_out[i, 2] = (
                vz * ct + (kx * vy - ky * vx) * st + kz * kv * (1 - ct) + tz
            )

        else:
            velo_out[i, :3] = vx + tx, vy + ty, vz + tz

        # intensity
        velo_out[i, 3] = velo_in[i, 3]

    return velo_out


def Rodrigues(matrix):
    """Convert the rotation matrix into the axis-angle notation.
    Conversion equations
    ====================
    From Wikipedia (http://en.wikipedia.org/wiki/Rotation_matrix), the conversion is given by::
        x = Qzy-Qyz
        y = Qxz-Qzx
        z = Qyx-Qxy
        r = hypot(x,hypot(y,z))
        t = Qxx+Qyy+Qzz
        theta = atan2(r,t-1)
    @param matrix:  The 3x3 rotation matrix to update.
    @type matrix:   3x3 numpy array
    @return:    The 3D rotation axis and angle.
    @rtype:     numpy 3D rank-1 array, float
    """

    # Axes.
    axis = np.zeros(3, np.float64)
    axis[0] = matrix[2, 1] - matrix[1, 2]
    axis[1] = matrix[0, 2] - matrix[2, 0]
    axis[2] = matrix[1, 0] - matrix[0, 1]

    # Angle.
    r = np.hypot(axis[0], np.hypot(axis[1], axis[2]))
    t = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    theta = np.arctan2(r, t - 1)

    # Normalise the axis.
    axis = axis / r

    # Return the data.
    return axis * theta


def interpolate_matrices(M1, M2, alpha):
    # Decompose matrices into translations and rotations
    translation1 = M1[:3, 3]
    translation2 = M2[:3, 3]
    rotation1 = R.from_matrix(M1[:3, :3])
    rotation2 = R.from_matrix(M2[:3, :3])

    # Linearly interpolate translations
    interpolated_translation = (1 - alpha) * translation1 + alpha * translation2

    # Spherically interpolate rotations
    slerp = Slerp([0, 1], R.concatenate([rotation1, rotation2]))
    interpolated_rotation = slerp([alpha])[0].as_matrix()

    # interpolated_rotation = R.from_rotvec(
    #     (1 - alpha) * rotation1.as_rotvec() + alpha * rotation2.as_rotvec()).as_matrix()

    # Reconstruct the 4x4 transformation matrix
    interpolated_matrix = np.eye(4)
    interpolated_matrix[:3, :3] = interpolated_rotation
    interpolated_matrix[:3, 3] = interpolated_translation
    return interpolated_matrix


def get_transformation(time_poses, t):
    if not time_poses:
        raise ValueError("time_poses dictionary is empty.")

    # Sort timestamps
    timestamps = sorted(time_poses.keys())

    if len(timestamps) < 2:
        raise ValueError("Not enough timestamps for interpolation.")

    # Find closest timestamps
    lower_timestamp = None
    upper_timestamp = None

    for timestamp in timestamps:
        if timestamp <= t:
            lower_timestamp = timestamp
        if timestamp >= t and upper_timestamp is None:
            upper_timestamp = timestamp

    # Handle cases where t is out of the bounds of the timestamps
    if lower_timestamp is None:
        return None
    if upper_timestamp is None:
        return None

    # Get transformation matrices for the two closest timestamps
    M1 = time_poses[lower_timestamp]
    M2 = time_poses[upper_timestamp]

    # Compute alpha for interpolation
    total_interval = upper_timestamp - lower_timestamp
    if total_interval > 0.15:
        return None
    alpha = (t - lower_timestamp) / total_interval if total_interval != 0 else 0

    # Interpolate between the two matrices
    return interpolate_matrices(M1, M2, alpha)


class PCDMerger:
    def __init__(self, pcd_dir, traj_file):
        self.pcd_dir = pcd_dir
        self.timestamps = self.get_timestamps()
        l2imu = self.get_l2imu()
        gt_imu2w = self.read_gt_traj(traj_file)
        self.gt_l2w = self.get_l2w(gt_imu2w, l2imu)
        # self.merge_pcd()

    def plot_traj(self, gt_imu2w):
        x = np.array([pose[0, 3] for pose in gt_imu2w])
        y = np.array([pose[1, 3] for pose in gt_imu2w])
        z = np.array([pose[2, 3] for pose in gt_imu2w])
        rpy = np.array(
            [
                R.from_matrix(pose[:3, :3]).as_euler("zyx", degrees=True)
                for pose in gt_imu2w
            ]
        )
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(x, y, z, label="trajectory")
        ax.legend()
        plt.show()
        # plt.figure()
        # plt.plot(np.arange(len(x)), rpy[:, 0], label="roll")
        # plt.show()
        # plt.plot(np.arange(len(x)), rpy[:, 1], label="pitch")
        # plt.show()
        # plt.plot(np.arange(len(x)), rpy[:, 3], label="yaw")
        # plt.show()
    def load_traj(self):
        poses = []
        for i, timestamp in enumerate(self.timestamps):
            if i not in self.gt_l2w:
                continue
            poses.append(self.gt_l2w[i])
        # self.plot_traj(poses)
        return poses
    def merge_pcd(self, start, end, pcd_station=None):
        global_map = o3d.geometry.PointCloud()
        poses = []
        for i in range(start, end):
            # print progess
            if i % 200 == 0:
                # global_map = global_map.voxel_down_sample(0.1)
                print(f"Processing frame {i}...")

            if i not in self.gt_l2w:
                # print(f"Skip timestamp {self.timestamps[i]}")
                continue

            pcd = self.get_lidar_pc(i)
            # pcd = self.get_lidar_pc_raw(i)
            pcd.transform(self.gt_l2w[i])
            poses.append(self.gt_l2w[i])
            global_map = self.merge_pcd_o3d(global_map, pcd)
        # self.plot_traj(poses)
        global_map.paint_uniform_color([0, 0.651, 0.929])
        global_map = global_map.voxel_down_sample(0.1)

        if pcd_station is None:
            return global_map
        # refine submap using station pcd
        np_raw_submap = np.asarray(global_map.points)
        kdtree = o3d.geometry.KDTreeFlann(pcd_station)
        indices = []
        for point in np_raw_submap:
            [_, idx, _] = kdtree.search_knn_vector_3d(point, 2)
            indices.extend(idx[1:])
        refine_submap = pcd_station.select_by_index(indices)
        refine_submap.paint_uniform_color([0, 0.651, 0.929])

        return refine_submap

    def merge_pcd_pointlio(self, start, end):
        global_map = o3d.geometry.PointCloud()
        for i in range(start, end):
            pcd = self.get_lidar_pc_raw(i)
            global_map = self.merge_pcd_o3d(global_map, pcd)
        return global_map
    def merge_pcd_SLAMS10(self, start, end, pcd_station=None):
        print("merge_pcd_SLAMS10...")
        global_map = o3d.geometry.PointCloud()
        for i in range(start, end):
            pcd = self.get_lidar_pc_raw(i)
            global_map = self.merge_pcd_o3d(global_map, pcd)
            if i % 300 == 299:
                print(f"downsample {i}")
                global_map = global_map.voxel_down_sample(0.01)
        if pcd_station is None:
            return global_map
        # refine submap using station pcd
        np_raw_submap = np.asarray(global_map.points)
        kdtree = o3d.geometry.KDTreeFlann(pcd_station)
        indices = []
        for point in np_raw_submap:
            [_, idx, _] = kdtree.search_knn_vector_3d(point, 10)
            indices.extend(idx[1:])
        refine_submap = pcd_station.select_by_index(indices)
        refine_submap.paint_uniform_color([0, 0.651, 0.929])
        # o3d.visualization.draw_geometries([refine_submap])
        return refine_submap
    def get_l2w(self, gt_imu2w: Dict, l2imu):
        gt_l2w = {}
        for i, timestamp in enumerate(self.timestamps):
            imu2w = get_transformation(gt_imu2w, timestamp)
            if imu2w is None:
                # print(f"GT: No pose found for timestamp {timestamp}")
                continue
            l2w = imu2w @ l2imu
            gt_l2w[i] = l2w
        return gt_l2w

    def get_timestamps(self):
        time_file = os.path.join(self.pcd_dir, "../timestamps.txt")
        timestamps = np.loadtxt(time_file, dtype=str)
        timestamps = [float(t) for t in timestamps]
        return timestamps

    def get_lidar_pc_raw(self, frame_id):
        pcd_path = self.pcd_dir + f"{frame_id:06d}.pcd"
        pcd = o3d.io.read_point_cloud(pcd_path)
        return pcd

    def get_lidar_pc(self, frame_id):
        pcd_path = self.pcd_dir + f"{frame_id:06d}.pcd"
        pcd = o3d.io.read_point_cloud(pcd_path)
        velo = np.asarray(pcd.points)
        ego_motion = self.get_ego_motion(frame_id)
        if np.linalg.norm(ego_motion - np.eye(4)) > 0.1:
            r = Rodrigues(ego_motion[0:3, 0:3])
            t = ego_motion[0:3, 3]
            velo = curl_velodyne_data(velo, r, t)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(velo)
        return pcd

    def get_ego_motion(self, frame_id):
        frame_ids = self.gt_l2w.keys()
        if frame_id in frame_ids and frame_id - 1 in frame_ids:
            return np.linalg.inv(self.gt_l2w[frame_id]) @ self.gt_l2w[frame_id - 1]
        elif frame_id in frame_ids and frame_id + 1 in frame_ids:
            return np.linalg.inv(self.gt_l2w[frame_id + 1]) @ self.gt_l2w[frame_id]
        else:
            return np.eye(4)

    def merge_pcd_o3d(
        self, pcd1: o3d.geometry.PointCloud, pcd2: o3d.geometry.PointCloud, ds_size=0.1
    ):
        # pcd2 = pcd2.voxel_down_sample(ds_size)
        pcd1 += pcd2
        return pcd1

    def get_l2imu(self):
        if cfg.seq == "bday":
            quat = [0.004028, -0.002505, 0.003725, 0.999982]
            # quat = [0.999982,  0.004028, -0.002505,  0.003725]
            xyz = np.array([-0.037354, 0.006824, -0.020190])
            rot_mat = R.from_quat(quat).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot_mat
            T[:3, 3] = xyz
        else:
            T = np.eye(4)
        return T

    def format_number(self, num):
        shift = 10 - math.floor(math.log10(abs(num))) - 1
        scaled_num = num * (10**shift)
        num_str = f"{scaled_num}"
        integer_part, decimal_part = num_str.split(".")

        decimal_part = decimal_part.rstrip("0")
        formatted_num = f"{integer_part}.{decimal_part}"
        return float(formatted_num)

    def read_gt_traj(self, traj_file):
        with open(traj_file, "r") as f:
            lines = f.readlines()
        traj = {}
        traj_list = []
        for i, line in enumerate(lines):
            line = line.strip().split()
            # time_stamp = float(line[0])
            time_stamp = self.format_number(float(line[0]))
            xyz = np.array([float(x) for x in line[1:4]])
            quaternion = [float(x) for x in line[4:8]]
            rot_mat = R.from_quat(quaternion).as_matrix()
            pose = np.eye(4)
            pose[:3, :3] = rot_mat
            pose[:3, 3] = xyz
            traj[time_stamp] = pose
            traj_list.append(pose)

        # self.plot_traj(traj_list)

        return traj
