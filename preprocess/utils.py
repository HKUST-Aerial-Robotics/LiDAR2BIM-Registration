import os
import numpy as np
import open3d as o3d
import numba as nb
from scipy.spatial.transform import Rotation as R
import math
from time import perf_counter
import matplotlib.pyplot as plt
import shutil
from tqdm import tqdm


class TicToc:
    def __init__(self):
        self.time_dict = {}

    def tic(self, key_name):
        if key_name not in self.time_dict:
            self.time_dict[key_name] = {"time": 0, "number": 0, "t0": perf_counter()}
        else:
            self.time_dict[key_name]["t0"] = perf_counter()

    def toc(self, key_name, verbose=False):
        if key_name not in self.time_dict:
            raise ValueError(f"No timer started for {key_name}")
        t1 = perf_counter()
        elapsed_time = (t1 - self.time_dict[key_name]["t0"]) * 1000
        self.time_dict[key_name]["time"] += elapsed_time
        self.time_dict[key_name]["number"] += 1
        if verbose:
            print(f"Time for {key_name}: {elapsed_time:.2f} ms")
        return elapsed_time

    def print_summary(self):
        for k, v in self.time_dict.items():
            average_time = v["time"] / v["number"] if v["number"] > 0 else 0
            print(f"Average Time for {k}: {average_time:.6f} ms")
    def get_avg(self, key_name):
        if key_name not in self.time_dict:
            raise ValueError(f"No timer started for {key_name}")
        return self.time_dict[key_name]["time"] / self.time_dict[key_name]["number"]
    def clear(self):
        self.time_dict = {}


class Stats:
    def __init__(self):
        self.stats_dict = {}

    def add(self, key_name, value):
        if key_name not in self.stats_dict:
            self.stats_dict[key_name] = []
        self.stats_dict[key_name].append(value)

    def get(self, key_name):
        if key_name not in self.stats_dict:
            raise ValueError(f"No stats recorded for {key_name}")
        return self.stats_dict[key_name]

    def print_summary(self):
        for k, v in self.stats_dict.items():
            avg = np.mean(v)
            print(f"Stats for {k}: {avg}")

    def clear(self):
        self.stats_dict = {}


class FileIO:
    def __init__(self):
        pass

    def not_black_pcd(self, pcd):
        # only select points that color is not black
        indices = np.where(np.asarray(pcd.colors)[:, 0] != 0)[0]
        return pcd.select_by_index(indices.tolist())

    def pcd_from_dir(self, pcd_dir, num=-1, start=0, wall=False, progress=False):
        pcds = []
        if num != -1:
            pcd_files = sorted(os.listdir(pcd_dir))[start : start + num]
        else:
            pcd_files = sorted(os.listdir(pcd_dir))
        if progress:
            pcd_files = tqdm(pcd_files)
        for pcd_file in pcd_files:
            pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, pcd_file))
            if wall:
                pcd = self.not_black_pcd(pcd)
            pcds.append(pcd)
        return pcds

    def lineseg_from_dir(self, lineseg_dir, num=-1):
        linesegs = []
        if num != -1:
            lineseg_files = sorted(os.listdir(lineseg_dir))[:num]
        else:
            # only select the txt files
            lineseg_files = []
            for file in sorted(os.listdir(lineseg_dir)):
                if file.endswith(".txt"):
                    lineseg_files.append(file)

            # lineseg_files = sorted(os.listdir(lineseg_dir))
        for lineseg_file in tqdm(lineseg_files):
            with open(os.path.join(lineseg_dir, lineseg_file), "r") as f:
                lineseg = f.readlines()
                linesegs.append(lineseg)
        return linesegs

    def pose_from_dir(self, pose_path):
        with open(pose_path, "r") as f:
            # do not read the first line
            poses = f.readlines()[1:]
        return poses


timer = TicToc()
stats = Stats()
io = FileIO()


def setup_plot(size):
    # # clear the plot
    # plt.clf()

    width = size[1] / 100
    height = size[0] / 100
    fig, ax = plt.subplots()
    fig.set_size_inches(width, height)
    ax = plt.Axes(fig, [0.0, 0.0, 1.0, 1.0])
    ax.axis("off")
    fig.add_axes(ax)
    return fig, ax


def svd_rot2d(X, Y, W=None):
    """
    Compute the optimal rotation matrix using SVD for weighted least squares fitting.

    Parameters:
    X (numpy.ndarray): 2xN matrix.
    Y (numpy.ndarray): 2xN matrix.
    W (numpy.ndarray): 1xN weight matrix.

    Returns:
    numpy.ndarray: 2x2 rotation matrix.
    """
    # Assemble the correlation matrix H = X * diag(W) * Y.T
    if W is None:
        W = np.ones(X.shape[1])
    H = X @ np.diag(W.flat) @ Y.T

    # Perform Singular Value Decomposition
    U, _, Vt = np.linalg.svd(H)

    # Compute rotation matrix V * U.T
    V = Vt.T
    if np.linalg.det(U) * np.linalg.det(V) < 0:
        V[:, 1] *= -1

    rotation_matrix = V @ U.T
    return rotation_matrix


def solve_2d_closedform(source: np.ndarray, target: np.ndarray):
    """
    Solve the 2D minimum problem using a closed-form solution for optimal rigid transformation,
    which includes translation and rotation.

    :param source: 2D numpy array (N, 3), where N is the number of points
    :param target: 2D numpy array (N, 3), where N is the number of points
    :return: tuple (tx, ty, yaw) representing the translation (tx, ty) and rotation (yaw in radians)
    """
    # Step 1: Center the points by subtracting their centroids
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid

    # Step 3: Compute the covariance matrix H
    H = np.dot(source_centered.T, target_centered)

    # Step 3: SVD of covariance matrix
    U, S, Vt = np.linalg.svd(H)
    V = Vt.T

    # Step 4: Compute the rotation matrix R
    R = np.dot(V, U.T)
    if np.linalg.det(R) < 0:
        # V[:, -1] *= -1
        # R = np.dot(V, U.T)
        # t = target_centroid - np.dot(R, source_centroid)
        #
        # yaw = np.arctan2(R[1, 0], R[0, 0])
        # yaw = np.degrees(yaw)
        # print("x, y, yaw", t[0], t[1], yaw)
        # print("source", source)
        # print("target", target)
        # fig, ax = plt.subplots()
        # ax.scatter(source[:, 0], source[:, 1], color="blue")
        # ax.scatter(target[:, 0], target[:, 1], color="red")
        # plt.show()
        return (None, None, None)


    # Step 5: Compute the translation vector t
    t = target_centroid - np.dot(R, source_centroid)

    # Step 6: Extract the rotation angle (yaw)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    yaw = np.degrees(yaw)

    return (t[0], t[1], yaw)


def solve_2d_triplets(source, target):
    # source is [A, B, C] and A, B, C are corners, to access coordinates of A, B, C, use A.x, A.y
    # target is [A', B', C'] and A', B', C' are corners, to access coordinates of A', B', C', use A'.x, A'.y
    # convert to numpy array
    np_src = np.array([[corner.x, corner.y] for corner in source])
    np_tgt = np.array([[corner.x, corner.y] for corner in target])
    return solve_2d_closedform(np_src, np_tgt)



def overlap_filtering(R, source, target, buffer=2.0):
    return True
    A, B, C = source
    A_, B_, C_ = target
    A_check = overlap_corner(A, A_,R, buffer=buffer)
    # B_check = overlap_corner(B, B_,R, buffer=buffer)
    # C_check = overlap_corner(C, C_,R, buffer=buffer)
    if A_check:
        return True
    return False
def overlap_corner(src_corner, tgt_corner, R, buffer=2.0):
    # A has four split lines, so has four directions
    src_dirs = np.array([src_corner.split_lines[i].direction for i in range(4)])
    tgt_dirs = np.array([tgt_corner.split_lines[i].direction for i in range(4)])

    src_length = [line.length for line in src_corner.split_lines]
    tgt_length = [line.length for line in tgt_corner.split_lines]

    src_dirs = R.dot(src_dirs.T).T
    dot_product = np.dot(src_dirs, tgt_dirs.T)
    max_indices = np.zeros(4, dtype=int)
    for i in range(dot_product.shape[0]):
        max_indices[i] = np.argmax(dot_product[i])

    # sort src_length by max_indices
    src_length = [src_length[i] for i in max_indices]
    # pair to pair comparison
    for i in range(4):
        if src_length[i] > tgt_length[i] + buffer:
            return False
    return True
def solve_2d_minimum(p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    # p1: (n, 2), p2: (n, 2), n = 2
    tim1 = p1[0] - p1[1]
    tim2 = p2[0] - p2[1]
    tim1 = tim1 / np.linalg.norm(tim1)
    tim2 = tim2 / np.linalg.norm(tim2)
    yaw = np.emath.arccos(np.dot(tim1, tim2))
    yaw = np.degrees(yaw)
    p1_ = p1 @ R.from_euler("z", yaw).as_matrix()[:2, :2].T
    x, y = np.mean(p2 - p1_, axis=0)
    return x, y, yaw


def random_transformation(scale=1.0):
    axis = np.random.rand(3)
    angle = np.random.rand() * 2 * np.pi
    axis = axis / np.linalg.norm(axis)
    t = np.random.rand(3) * scale
    rot_mat = R.from_rotvec(axis * angle).as_matrix()
    transformation = np.eye(4)
    transformation[:3, :3] = rot_mat
    transformation[:3, 3] = t
    return transformation


def rotation_to_yaw(rotation, degrees=False):
    r = R.from_matrix(rotation)
    return r.as_euler("zyx", degrees=degrees)[0]


def xyyaw_to_se3(x, y, yaw, degrees=False):
    se3 = np.eye(4)
    se3[:3, :3] = R.from_euler("z", yaw, degrees=degrees).as_matrix()
    se3[:3, 3] = np.array([x, y, 0])
    return se3


def se32xyyaw(se3, degrees=False):
    """
    :param se3: 4x4 matrix
    :return: x, y, yaw
    """
    x, y = se3[:2, 3]
    yaw = rotation_to_yaw(se3[:3, :3], degrees=degrees)
    return x, y, yaw


def se32se2(se3):
    """
    :param se3: 4x4 matrix
    :return: se2: 4x4 matrix
    """
    yaw = rotation_to_yaw(se3[:3, :3], degrees=True)
    se2 = np.eye(4)
    se2[:3, :3] = R.from_euler("z", yaw, degrees=True).as_matrix()
    se2[:2, 3] = se3[:2, 3]
    return se2


def mkdir(path):
    if not os.path.exists(path):
        print(f"Creating directory: {path}")
        os.makedirs(path)


def mkdirs(paths):
    if not isinstance(paths, (list, tuple)):
        paths = [paths]
    for path in paths:
        mkdir(path)


def clear_dir(path):
    print(f"Clearing directory: {path}")
    for item in os.listdir(path):
        item_path = os.path.join(path, item)
        if os.path.isfile(item_path):
            os.remove(item_path)
        elif os.path.isdir(item_path):
            shutil.rmtree(item_path)


def clear_dirs(paths):
    if not isinstance(paths, (list, tuple)):
        paths = [paths]
    for path in paths:
        clear_dir(path)


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


def remove_ground(pc: np.ndarray):
    """
    Remove ground points from point cloud N*3
    """
    N_SCAN = 64
    Horizon_SCAN = 1800
    ang_res_x = 360.0 / Horizon_SCAN
    ang_res_y = 26.9 / (N_SCAN - 1)
    ang_bottom = 25.0
    sensor_mount_angle = 0.0  # Assuming sensor is mounted horizontally.

    ground_mat = np.zeros((N_SCAN, Horizon_SCAN))
    ground_mat.fill(-1)  # Initial value for all pixels is -1 (no info).

    range_image = np.zeros((N_SCAN, Horizon_SCAN, pc.shape[1]))
    for i in range(pc.shape[0]):
        vertical_angle = (
            np.arctan2(pc[i, 2], np.sqrt(pc[i, 0] ** 2 + pc[i, 1] ** 2)) * 180 / np.pi
        )
        row_id = (vertical_angle + ang_bottom) / ang_res_y
        if row_id < 0 or row_id >= N_SCAN:
            continue
        horizon_angle = np.arctan2(pc[i, 0], pc[i, 1]) * 180 / np.pi
        column_id = -np.round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN / 2
        if column_id >= Horizon_SCAN:
            column_id -= Horizon_SCAN
        if column_id < 0 or column_id >= Horizon_SCAN:
            continue
        range_pc = np.sqrt(pc[i, 0] ** 2 + pc[i, 1] ** 2 + pc[i, 2] ** 2)
        if range_pc < 2:
            continue
        range_image[int(row_id), int(column_id), :] = pc[i, :]

    for j in range(Horizon_SCAN):
        for i in range(1, N_SCAN):
            if np.any(range_image[i - 1, j, :] == 0) or np.any(
                range_image[i, j, :] == 0
            ):
                continue  # No info to check, invalid points.

            diff = range_image[i, j, :] - range_image[i - 1, j, :]
            angle = (
                np.arctan2(diff[2], np.sqrt(diff[0] ** 2 + diff[1] ** 2)) * 180 / np.pi
            )

            if np.abs(angle - sensor_mount_angle) <= 10:
                ground_mat[i - 1, j] = 1
                ground_mat[i, j] = 1

    # Extract ground cloud (groundMat == 1).
    ground_cloud = []
    for i in range(N_SCAN):
        for j in range(Horizon_SCAN):
            if ground_mat[i, j] == 1:
                ground_cloud.append(range_image[i, j, :])

    # Apply the mask to the range image to get the non-ground points.
    non_ground_cloud = range_image[ground_mat[:, :] != 1]

    return non_ground_cloud


def create_point_cloud(points, color=(0, 0.651, 0.929)):
    xyz = points[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.paint_uniform_color(color)
    return pcd


def draw_pairs(src, dst, tf, window_name="pairs"):
    src = create_point_cloud(src, color=[0, 0.651, 0.929])
    dst = create_point_cloud(dst, color=[1.000, 0.706, 0.000])
    src = src.transform(tf)
    o3d.visualization.draw_geometries([src, dst], window_name=window_name)


def draw_pc(pc, window_name="pc"):
    pc = create_point_cloud(pc, color=[0, 0.651, 0.929])
    o3d.visualization.draw_geometries([pc], window_name=window_name)


def average_angle(angles):
    """
    Compute the average of a list of angles, taking into account their circular nature.

    :param angles: List or array of angles in degrees.
    :return: Average angle in degrees, normalized between [-180, 180).
    """
    # Convert angles to radians
    angles_rad = np.radians(angles)

    # Compute the sum of the unit vectors represented by the angles
    sum_sin = np.sum(np.sin(angles_rad))
    sum_cos = np.sum(np.cos(angles_rad))

    # Compute the average angle
    avg_angle_rad = np.arctan2(sum_sin, sum_cos)
    return avg_angle_rad


def angle_diff(a, b):
    diff = (a - b + 180) % 360 - 180
    return abs(diff)


def find_angle_inliers(angles_deg, thd):
    """
    Find the inlier set of angles.
    """

    best_inliers = []
    for select_angle in angles_deg:
        inlies = [
            angle
            for angle in angles_deg
            if min(
                angle_diff(angle, select_angle), angle_diff(angle + 180, select_angle)
            )
            < thd
        ]
        if len(inlies) > len(best_inliers):
            best_inliers = inlies
        if len(best_inliers) > len(angles_deg) // 2:
            break
    return best_inliers


def average_vectors(vectors):
    # Convert vectors to angles in radians
    angles_rad = np.arctan2([v[1] for v in vectors], [v[0] for v in vectors])
    angles_deg = np.rad2deg(angles_rad)
    for i, angle in enumerate(angles_deg):
        if angle_diff(angle, angles_deg[0]) > angle_diff(angle + 180, angles_deg[0]):
            angles_deg[i] = angle + 180

    avg_angle_rad = average_angle(angles_deg)

    return np.array([np.cos(avg_angle_rad), np.sin(avg_angle_rad)])


def icp_registration(source, target, init_tf, voxel_size=0.3):
    # point to plane
    source_pcd = create_point_cloud(source)
    target_pcd = create_point_cloud(target)
    source_down = source_pcd.voxel_down_sample(voxel_size)
    target_down = target_pcd.voxel_down_sample(voxel_size)

    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    loss = o3d.pipelines.registration.HuberLoss(k=0.5)
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source_down,
        target_down,
        0.1,
        init_tf,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    return reg_p2l.transformation
if __name__ == "__main__":
    # test solve_2d_closedform
    source = np.array([[0, 0], [1, 0], [0, 1]])
    target = np.array([[0, 0], [-1, 0], [0, -1]])
    timer.tic("closedform")
    t = solve_2d_closedform(source, target)
    timer.toc("closedform")
    print(t)
    timer.print_summary()

    # R = np.array([
    #     [-1, 0, 0],
    #     [0, -1, 0],
    #     [0, 0, 1]
    # ])

    # # Calculate yaw angle
    # yaw = np.arctan2(R[1, 0], R[0, 0])
    # print(yaw)
    # yaw_degrees = np.degrees(yaw)
    # print(yaw_degrees)