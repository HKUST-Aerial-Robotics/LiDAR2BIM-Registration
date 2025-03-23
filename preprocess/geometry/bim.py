import os
import open3d as o3d
from preprocess.geometry.lineseg import LineSegments


class BaseMapManager:
    def __init__(self, cfg):
        self.cfg = cfg
        self.hash_table = None
        self.occupied_pcd = None
        self.lines: LineSegments = None

        self.pc_manager = None
        self.raster = None

    def load_pcd(self, path, color=(0.929, 0, 0.651)):
        if os.path.exists(path):
            pcd = o3d.io.read_point_cloud(path)
        else:
            raise FileNotFoundError(f"pcd not found, copy bim.pcd to the {path} ")
        pcd.paint_uniform_color(color)
        return pcd

    def load_lines(self, linesegs_file) -> LineSegments:
        if os.path.exists(linesegs_file):
            linesegs = LineSegments([])
            linesegs.read_from_file(linesegs_file)
        else:
            raise FileNotFoundError(
                f"LineSeg.txt not found, copy bimLineSeg.txt to the {linesegs_file} "
            )
        return linesegs

    def construct_hash_table(self):
        print("Constructing hash table...")
        corners = self.extract_corners()
        self.pc_manager.extract_hash_descriptors(corners)
        self.hash_table = self.pc_manager.desc_hashtable
        return self.hash_table

    def save_hash_table(self, file_path):
        self.hash_table.save_to_file(file_path)



    def extract_corners(self):
        self.lines.extend(dist=2)
        self.corners = self.lines.intersections(threshold=0.7)
        self.lines.extend(dist=-2)

        return self.corners


class BIMManager(BaseMapManager):
    def __init__(self, cfg):
        super(BIMManager, self).__init__(cfg)
        self.subbim_dict = {}

        self.bim_dir = os.path.join(cfg.ROOT_DIR, "data/LiBIM-UST/BIM", cfg.floor)
        self.subbim_dir = os.path.join(self.bim_dir, "subBIMs")

    def load_bim(self, load_hash_table=True, load_lines=True):
        # pcd
        pcd_file = os.path.join(self.bim_dir, "bim.pcd")
        self.pcd = self.load_pcd(pcd_file)
        pcd2d_file = os.path.join(self.bim_dir, "bim2d.pcd")
        self.pcd_proj = self.load_pcd(pcd2d_file)

    def load_pcd(self, path, color=(0.6, 0.6, 0.6)):
        if os.path.exists(path):
            pcd = o3d.io.read_point_cloud(path)
        else:
            raise FileNotFoundError(f"bim_pcd not found, copy bim.pcd to the {path} ")
        pcd.paint_uniform_color(color)
        return pcd