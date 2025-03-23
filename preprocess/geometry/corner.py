import open3d as o3d
from typing_extensions import Self
from preprocess.utils import timer


class Corner:
    def __init__(self, x, y):
        x = round(x, 2)
        y = round(y, 2)
        self.x = x
        self.y = y
        self.lines = None
        self.split_lines = None

    def __str__(self):
        return f"Corner({self.x}, {self.y})"

    def to_o3d(self, color=[0,  0.0, 0.1], radius=0.1):
        # return sphere
        # sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        box = o3d.geometry.TriangleMesh.create_box(width=radius*2, height=radius*2, depth=radius)
        box.compute_vertex_normals()
        box.translate([self.x -radius, self.y -radius, 0])
        box.paint_uniform_color(color)
        return box

    def set_lines(self, lines):
        self.lines = lines
    def set_split_lines(self, split_lines):
        self.split_lines = split_lines
    def distance(self, other: Self):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def get_xy(self):
        return self.x, self.y


if __name__ == "__main__":
    import numpy as np

    A = np.array([
        [-0.395901, 0.918293],
        [-0.928969, -0.370158],
        [0.388259, -0.921550],
        [0.927226, 0.374503]
    ])

    B = np.array([
        [0, -1],
        [0, 1],
        [-1, 0],
        [1, 0]
    ])
    rotation_degree = -112.59581668603512
    rotation_rad = np.radians(rotation_degree)
    rotation_matrix = np.array([
        [np.cos(rotation_rad), -np.sin(rotation_rad)],
        [np.sin(rotation_rad), np.cos(rotation_rad)]
    ])
    # rotate A
    timer.tic("dot_product")
    A = rotation_matrix.dot(A.T).T

    dot_product_matrix = np.dot(A, B.T)

    max_indices = np.zeros(A.shape[0], dtype=int)

    for i in range(dot_product_matrix.shape[0]):
        max_indices[i] = np.argmax(dot_product_matrix[i])
    timer.toc("dot_product")
    print("A after rotation:", A)
    print("每行的最大点积值的索引:", max_indices)
    timer.print_summary()