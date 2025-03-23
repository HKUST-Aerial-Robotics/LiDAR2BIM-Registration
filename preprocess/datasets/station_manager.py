import os
import open3d as o3d


class StationManager:
    def __init__(self, cfg):
        LiBIM_UST_ROOT = os.getenv("LiBIM_UST_ROOT")
        if LiBIM_UST_ROOT:
            pass
        else:
            print("LiBIM_UST_ROOT is not set！")
        self.station_dir = os.path.join(
            LiBIM_UST_ROOT, cfg.seq, "station"
        )

        path = os.path.join(self.station_dir, "station_2000k.ply")
        if os.path.exists(path):
            self.pcd = o3d.io.read_point_cloud(path)
        else:
            raise FileNotFoundError(
                f"station_pcd not found, copy station_2000k.ply to the {path} "
            )
        self.pcd.paint_uniform_color([0.929, 0.651, 0])
