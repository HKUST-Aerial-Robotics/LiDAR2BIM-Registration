import cv2
import numpy as np
import open3d as o3d
from preprocess.utils import timer
from copy import deepcopy
from typing import List
from typing_extensions import Self
from preprocess.geometry.corner import Corner
from preprocess.utils import svd_rot2d
from preprocess.clustering_utils import UnionFind
from tqdm import tqdm
from shapely.geometry import LineString, Polygon


class LineSegmentMerger:
    def __init__(self, cfg, linesegs_list=None):
        self.cfg = cfg
        if linesegs_list is None:
            self.linesegs = LineSegments([])
            self.obs_list = []
        else:
            self.linesegs = linesegs_list[0]
            self.obs_list = linesegs_list[1:]

    def extract(self, pcd2d_list):
        print("Extracting line segments...")
        linesegs_list = []
        for pcd2d in tqdm(pcd2d_list):
            extractor = LineSegmentExtractor(walls_2d=pcd2d.walls_2d)
            extractor.extract_lineseg()
            linesegs_list.append(extractor.linesegs)
        self.linesegs = linesegs_list[0]
        self.obs_list = linesegs_list[1:]

    def merge(self, angle_thd=5, pt_thd=0.2, minor_thd=2, short_thd=0.6):
        print("Merging line segments...")
        i = 0
        for obs in tqdm(self.obs_list):
            if i % 10 == 0:
                self.linesegs.merge_uf(angle_thd=angle_thd, pt_thd=pt_thd)
            self.linesegs.merge(obs, angle_thd=angle_thd, pt_thd=pt_thd)
            i += 1
        self.linesegs.remove_minor_obs(thd=minor_thd)
        self.linesegs.remove_short(thd=short_thd)
        self.linesegs.cluster_angle(
            bandwidth=0.1
        )  # rectify the direction by mean shift
        # self.linesegs.remove_minor_angle(min_num=3, min_length=2.0)
        self.linesegs.merge_uf(angle_thd=angle_thd, pt_thd=pt_thd)
        return self.linesegs

    def get_o3d_lineset(self) -> List[o3d.geometry.LineSet]:
        lineset_list = []
        linesegs_list = [self.linesegs] + self.obs_list
        for lineseg in linesegs_list:
            lineset_list.append(lineseg.get_o3d_lineset())
        return lineset_list


class LineSegmentExtractor:
    def __init__(self, walls_2d=None, wall_clusters_2d=None):
        self.walls_2d = walls_2d
        self.wall_clusters_2d = wall_clusters_2d
        self.linesegs = None

    def extract_lineseg(self):
        timer.tic("extract_seg o3d")
        wall_clusters = []
        lineseg_list = []
        self.walls_2d.voxel_down_sample(voxel_size=0.1)
        if self.wall_clusters_2d is None:
            colors_rounded = np.round(np.array(self.walls_2d.colors), decimals=8)
            unique_walls_colors = np.unique(colors_rounded, axis=0)
            for color in unique_walls_colors:
                color_mask = np.all(colors_rounded == color, axis=1)
                indices = np.where(color_mask)[0]
                if len(indices) < 4:
                    continue
                cluster = self.walls_2d.select_by_index(indices)
                # refined_clusters = [cluster]
                max = np.max(np.array(cluster.points))
                min = np.min(np.array(cluster.points))
                length = np.linalg.norm(max - min)
                if length / len(cluster.points) < 0.3:
                    refined_clusters = [cluster]
                else:
                    refined_clusters = self.spatial_cluster(
                        cluster, distance_threshold=0.8
                    )
                wall_clusters.extend(refined_clusters)
        else:
            wall_clusters = self.wall_clusters_2d
        timer.tic("find_endpoints by fitting")
        for cluster in wall_clusters:
            min_pt, max_pt = find_endpoints(np.array(cluster.points)[:, :2])
            lineseg_list.append(LineSegment(min_pt, max_pt))
        self.linesegs = LineSegments(lineseg_list)
        timer.toc("find_endpoints by fitting")
        timer.toc("extract_seg o3d")

    def spatial_cluster(self, pcd, distance_threshold):
        points = np.asarray(pcd.points)
        num_points = len(points)
        uf = UnionFind(num_points)

        dist_matrix = np.linalg.norm(
            points[:, np.newaxis] - points[np.newaxis, :], axis=2
        )

        for i in range(num_points):
            for j in range(i + 1, num_points):
                if dist_matrix[i, j] < distance_threshold:
                    uf.union(i, j)

        clusters = {}
        for point_idx in range(num_points):
            root = uf.find(point_idx)
            if root in clusters:
                clusters[root].append(point_idx)
            else:
                clusters[root] = [point_idx]

        clustered_point_clouds = []
        for indices in clusters.values():
            if len(indices) >= 2:  # 可以设置一个最小大小的阈值
                cluster = pcd.select_by_index(indices)
                clustered_point_clouds.append(cluster)

        return clustered_point_clouds


class LineSegment:
    def __init__(self, point_a=None, point_b=None):
        self.point_a = point_a
        self.point_b = point_b
        if point_a is None or point_b is None:
            self.direction = np.random.rand(2)
            self.length = 0
        else:
            self.direction = self.point_b - self.point_a
            # ensure the direction is float
            self.direction = self.direction.astype(float)
            self.direction /= np.linalg.norm(self.direction)
            self.length = self.get_length()
        self.obs = LineSegments([self])


    def __str__(self):
        return f"LineSegment: [{self.point_a[0]:.2f}, {self.point_a[1]:.2f}]-> [{self.point_b[0]:.2f}, {self.point_b[1]:.2f}]"

    def sample_points(self, resolution=0.1):
        length = np.linalg.norm(self.point_a - self.point_b)
        num = int(length / resolution)
        return np.linspace(self.point_a, self.point_b, num)

    def random(self) -> Self:
        self.point_a = np.random.rand(2)
        self.point_b = np.random.rand(2)
        self.direction = self.point_b - self.point_a
        self.direction /= np.linalg.norm(self.direction)
        return self

    def transform(self, rot: np.ndarray, t: np.ndarray) -> Self:
        self.point_a = rot[:2, :2] @ self.point_a + t[:2]
        self.point_b = rot[:2, :2] @ self.point_b + t[:2]
        self.direction = rot[:2, :2] @ self.direction
        return self

    def from_corners(self, corner_a: Corner, corner_b: Corner):
        self.point_a = np.array([corner_a.x, corner_a.y])
        self.point_b = np.array([corner_b.x, corner_b.y])
        self.direction = self.point_b - self.point_a
        self.direction /= np.linalg.norm(self.direction)
        return self

    def get_midpoint(self):
        return (self.point_a + self.point_b) / 2

    def from_midpoint(self, midpoint, direction, length):
        self.point_a = midpoint - direction * length / 2
        self.point_b = midpoint + direction * length / 2
        self.direction = direction
        return self

    def get_corners(self):
        points = np.vstack([self.point_a.reshape(1, -1), self.point_b.reshape(1, -1)])
        return points

    def get_o3d_lineset(self):
        points = np.vstack([self.point_a, self.point_b])
        lines = [[0, 1]]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set

    def get_length(self):
        return np.linalg.norm(self.point_a - self.point_b)

    def similar(self, other: Self, angle_thd=5, pt_thd=0.2):
        if (
            other.distance(self.point_a) < pt_thd
            or other.distance(self.point_b) < pt_thd
            or self.distance(other.point_a) < pt_thd
            or self.distance(other.point_b) < pt_thd
        ):
            cos_sim = np.dot(self.direction, other.direction)
            if abs(cos_sim) > np.cos(np.deg2rad(angle_thd)): # for draft
            # if cos_sim > np.cos(np.deg2rad(angle_thd)): # real for dataset
                return True
        return False

    def distance(self, point: np.ndarray) -> float:
        """
        Calculate the minimum distance from a point to the line segment.

        Parameters:
        - point: np.ndarray, the point for which the distance is to be calculated

        Returns:
        - float, the distance from the point to the line segment
        """
        if np.array_equal(self.point_a, self.point_b):
            # If the segment is actually a point, return the distance to the point
            return np.linalg.norm(point - self.point_a)

        # Vector from point_a to the point
        ap = point - self.point_a
        # Project vector ap onto the line's direction
        proj_length = np.dot(ap, self.direction)
        # Calculate the closest point projection on the line segment
        closest_point = self.point_a + proj_length * self.direction

        # Check if the projection falls on the line segment
        if proj_length < 0:
            # Closest to point_a
            closest_point = self.point_a
        elif proj_length > np.linalg.norm(self.point_b - self.point_a):
            # Closest to point_b
            closest_point = self.point_b

        # The distance from the point to the closest point on the line segment
        return np.linalg.norm(point - closest_point)

    def angle(self, other: Self):
        return np.rad2deg(
            np.arccos(abs(np.clip(np.dot(self.direction, other.direction), -1.0, 1.0)))
        )

    def update(self, other: Self, avg=True):
        if avg:
            new_line = LineSegments([self, other]).average()
            self.reset(new_line)
        # if len(self.obs) > 30:
        #     self.obs = LineSegments([self])
        self.obs.append(other)

    def reset(self, other: Self):
        self.point_a = other.point_a
        self.point_b = other.point_b
        self.direction = other.direction


class LineSegments:
    def __init__(self, linesegments: List[LineSegment]):
        self.linesegments = linesegments
        self.uf = UnionFind(len(linesegments))

    def __str__(self):
        return f"LineSegments: {len(self.linesegments)}"

    def __iter__(self):
        for line in self.linesegments:
            yield line

    def __len__(self):
        return len(self.linesegments)

    def __getitem__(self, item):
        return self.linesegments[item]

    def append(self, line: LineSegment):
        self.linesegments.append(line)

    def crop(self, polygon: List[List[float]]):
        new_linesegs = []
        # using shapely to crop the linesegments, the region is a polygon
        polygon = Polygon(polygon)
        for line in self.linesegments:
            line_str = LineString([line.point_a, line.point_b])
            if polygon.intersects(line_str):
                new_linesegs.append(line)
        return LineSegments(new_linesegs)
    def sample_points(self, resolution=0.1):
        points = []
        for line in self.linesegments:
            points.extend(line.sample_points(resolution))
        return np.array(points)
    def extend(self, dist=0.1):
        new_linesegs = []
        for line in self.linesegments:
            new_line = deepcopy(line)
            new_line.point_a -= dist * new_line.direction
            new_line.point_b += dist * new_line.direction
            new_line.length = new_line.get_length()
            new_linesegs.append(new_line)
        self.linesegments = LineSegments(new_linesegs)

    def intersections(self, threshold=0.1) -> List[Corner]:
        corners = []
        for i in range(len(self)):
            for j in range(i + 1, len(self)):
                line1 = self[i]
                line2 = self[j]
                if np.array_equal(line1.point_a, line1.point_b) or np.array_equal(
                    line2.point_a, line2.point_b
                ):
                    continue
                x1, y1 = line1.point_a
                x2, y2 = line1.point_b
                x3, y3 = line2.point_a
                x4, y4 = line2.point_b
                if (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) == 0:
                    continue
                x = (
                    (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
                ) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
                y = (
                    (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)
                ) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
                if (
                    min(x1, x2)-0.001 <= x <= max(x1, x2)+0.001
                    and min(x3, x4)-0.001 <= x <= max(x3, x4)+0.001
                    and min(y1, y2) -0.001<= y <= max(y1, y2)+0.001
                    and min(y3, y4) -0.001<= y <= max(y3, y4)+0.001
                ):
                    corner = Corner(x, y)
                    corner.set_lines(LineSegments([line1, line2]))
                    corners.append(corner)
        # check identical corners,  remove the duplicate ones, if the distance is less than 0.1

        to_remove = set()
        corners_length = len(corners)

        for i in range(corners_length):
            for j in range(i + 1, corners_length):
                if corners[i].distance(corners[j]) < threshold:
                    to_remove.add(corners[j])

        new_corners = [corner for corner in corners if corner not in to_remove]
        corners = new_corners

        for corner in corners:
            corner_xy = np.array(corner.get_xy())
            split_lines = [
                              LineSegment(point_a=corner_xy, point_b=line.point_a) for line in corner.lines
                          ] + [
                              LineSegment(point_a=corner_xy, point_b=line.point_b) for line in corner.lines
                          ]
            corner.set_split_lines(LineSegments(split_lines))

        return corners

    def registration(self, lines_dst: Self):
        assert len(self) == len(lines_dst)

        directions_src = np.array([line.direction for line in self]).T
        directions_dst = np.array([line.direction for line in lines_dst]).T
        rot_est = svd_rot2d(directions_src, directions_dst)

        points_src = np.array([line.point_a for line in self]).T
        points_src_transformed = rot_est[:2, :2] @ points_src
        points_tgt = np.array([line.point_a for line in lines_dst]).T

        A = []
        b = []
        for i in range(len(lines_dst)):
            d = lines_dst[i].direction
            A_ = np.eye(2) - d @ d.T
            b_ = -A_ @ (points_src_transformed[:, i] - points_tgt[:, i])
            A.append(A_)
            b.append(b_)
        A = np.concatenate(A, axis=0)
        b = np.concatenate(b, axis=0)
        t_est = np.linalg.inv(A.T @ A) @ A.T @ b

        tf = np.eye(3)
        tf[:2, :2] = rot_est
        tf[:2, 2] = t_est
        return tf

    def transform(self, rot: np.ndarray, t: np.ndarray) -> Self:
        for line in self:
            line.transform(rot, t)
        return self

    def average(self):
        # print([line.get_length() for line in self])
        points = [line.sample_points() for line in self]
        points = np.concatenate(points, axis=0)
        # print(points.shape)
        # Sample data points
        x = points[:, 0]
        y = points[:, 1]
        if len(x) < 2:
            return self[0]

        # Fit the data to a line (1st degree polynomial)
        slope, intercept = np.polyfit(x, y, 1)

        def line(x):
            return slope * x + intercept

        point_a = np.array([np.min(x), line(np.min(x))])
        point_b = np.array([np.max(x), line(np.max(x))])
        return LineSegment(point_a, point_b)

    def merge(self, obs_lines: Self, angle_thd=5, pt_thd=1):
        new_line = [True for _ in range(len(obs_lines))]
        timer.tic("similar")
        for line in self:
            for i, obs in enumerate(obs_lines):
                if line.similar(obs, angle_thd=angle_thd, pt_thd=pt_thd):
                    if obs.get_length() > 0.2:
                        line.update(obs, avg=False)
                        new_line[i] = False
        timer.toc("similar")
        timer.tic("append")
        for i, obs in enumerate(obs_lines):
            if new_line[i] and obs.get_length() > 0.2:
                self.append(obs)
        timer.toc("append")
        timer.tic("average")
        for line in self:
            if len(line.obs) > 1:
                new_line = line.obs.average()
            else:
                new_line = line.obs[0]
            line.reset(new_line)
        timer.toc("average")

    # 利用UnionFind做更进一步的NMS
    def merge_uf(self, angle_thd=5, pt_thd=1):
        self.uf = UnionFind(len(self))
        for i in range(len(self)):
            for j in range(i + 1, len(self)):
                if self[i].similar(self[j], angle_thd=angle_thd, pt_thd=pt_thd):
                    self.uf.union(i, j)
        root_map = {}
        for i in range(len(self)):
            root = self.uf.find(i)
            if root not in root_map:
                root_map[root] = self[i]
            else:
                root_map[root].update(self[i])
        self.linesegments = list(root_map.values())

    def remove_minor_obs(self, thd=10):
        # remove the linesegments with length of observation less than thd
        self.linesegments = [line for line in self.linesegments if len(line.obs) > thd]

    def remove_short(self, thd=0.5):
        # remove the linesegments with length less than thd
        self.linesegments = [
            line for line in self.linesegments if line.get_length() > thd
        ]

    def mean_shift_step(self, bandwidth):
        new_directions = []
        for i, seg_i in enumerate(self.linesegments):
            numerator = np.zeros(2)
            denominator = 0
            for seg_j in self.linesegments:
                distance = angle_between(seg_i.direction, seg_j.direction)
                kernel = gaussian_kernel(distance, bandwidth)  # 或尝试使用其他核函数
                numerator += kernel * seg_j.direction
                denominator += kernel
            new_direction = (
                numerator / denominator if denominator != 0 else seg_i.direction
            )
            new_direction /= np.linalg.norm(
                new_direction
            )  # Normalize the new direction
            new_directions.append(new_direction)

        # Update directions
        for dir, seg in zip(new_directions, self.linesegments):
            seg.direction = dir

    def cluster_angle(self, bandwidth=0.1, tolerance=1e-5, max_iter=10):
        """Cluster line segments based on the angle of their direction vectors using mean shift."""
        for _ in range(max_iter):
            old_directions = [np.copy(seg.direction) for seg in self.linesegments]
            self.mean_shift_step(bandwidth)
            # Check convergence
            converged = True
            for old_dir, seg in zip(old_directions, self.linesegments):
                if np.linalg.norm(seg.direction - old_dir) > tolerance:
                    converged = False
                    break
            if converged:
                break
        # update the line segments by its direction and length and midpoint
        for seg in self.linesegments:
            midpoint = seg.get_midpoint()
            seg.from_midpoint(midpoint, seg.direction, seg.get_length())

    def remove_minor_angle(self, min_num=3, min_length=1.0):
        cluster_centers = []
        for line in self.linesegments:
            cluster_centers.append(line.direction)
        cluster_centers = np.array(cluster_centers)
        cluster_centers = np.unique(cluster_centers, axis=0)

        indices_to_remove = []
        for cluster_center in cluster_centers:
            indices = []
            for i, line in enumerate(self.linesegments):
                if np.linalg.norm(line.direction - cluster_center) < 0.002:
                    indices.append(i)
            if len(indices) < min_num or len(indices) < 0.1 * len(self.linesegments):
                for index in indices:
                    if self.linesegments[index].get_length() < min_length:
                        indices_to_remove.append(index)

        indices_to_remove = list(set(indices_to_remove))
        # print(f"Remove {len(indices_to_remove)} linesegments")
        for index in sorted(indices_to_remove, reverse=True):
            self.linesegments.pop(index)

    def get_o3d_lineset(self):
        points = []
        lines = []
        index = 0
        for linesegment in self.linesegments:
            p1 = np.hstack([linesegment.point_a, 0])
            p2 = np.hstack([linesegment.point_b, 0])
            points.extend([p1, p2])
            lines.append([index, index + 1])
            index += 2

        points = np.array(points)
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set

    def get_2d_np_array(self):
        lines = []
        for line in self.linesegments:
            lines.append([[line.point_a[0], line.point_a[1]], [line.point_b[0], line.point_b[1]]])
        return np.array(lines, dtype=np.int32)
    def get_3d_np_array(self):
        lines = []
        for line in self.linesegments:
            lines.append([[line.point_a[0], line.point_a[1], 0], [line.point_b[0], line.point_b[1], 0]])
        return np.array(lines)
    def read_from_file(self, file_path):
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                line = line.strip().split(" ")
                point_a = np.array([float(line[0]), float(line[1])])
                point_b = np.array([float(line[2]), float(line[3])])
                self.linesegments.append(LineSegment(point_a, point_b))
        return self

    def save_to_file(self, file_path):
        with open(file_path, "w") as f:
            for line in self.linesegments:
                f.write(
                    f"{line.point_a[0]} {line.point_a[1]} {line.point_b[0]} {line.point_b[1]}\n"
                )


def fit_line_cv2(pts):
    pts = pts.astype(np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0.1, 0.01, 0.01)
    mean_pt = np.mean(pts, axis=0)
    v = np.array([vx, vy]).flatten()
    return mean_pt, v


# def fit_line_ransac(pts, min_samples=2, residual_threshold=0.1, max_trials=1000):
#     model_robust, inliers = ransac(
#         pts,
#         LineModelND,
#         min_samples=min_samples,
#         residual_threshold=residual_threshold,
#         max_trials=max_trials,
#     )
#
#     origin, direction = model_robust.params
#     return origin, direction


def find_endpoints(pts):
    # # timer.tic("fit_line ransac")
    # origin, v = fit_line_ransac(pts, max_trials=20)
    # # timer.toc("fit_line ransac")
    timer.tic("fit_line cv2")
    origin, v = fit_line_cv2(pts)
    timer.toc("fit_line cv2")

    proj = np.dot(pts - origin, v)
    min_index = np.argmin(proj)
    max_index = np.argmax(proj)

    min_pt = origin + proj[min_index] * v
    max_pt = origin + proj[max_index] * v

    return min_pt, max_pt


def angle_between(u, v):
    """Calculate the absolute angle between two vectors."""
    return np.arccos(np.clip(np.dot(u, v), -1.0, 1.0))


def gaussian_kernel(distance, bandwidth):
    """Calculate weights using a Gaussian kernel."""
    return np.exp(-0.5 * (distance / bandwidth) ** 2)
