import os


class DirManager:
    def __init__(self, cfg):
            # self.data_dir = os.path.join(
            #     cfg.ROOT_DIR, "data/LiBIM-UST-raw", cfg.seq
            # )  # raw data
            # self.seq_dir = os.path.join(
            #     cfg.ROOT_DIR, "data/LiBIM-UST", cfg.seq, f"{cfg.interval:02d}m"
            # )
        LiBIM_UST_ROOT = os.getenv("LiBIM_UST_ROOT")
        if LiBIM_UST_ROOT:
            pass
        else:
            print("LiBIM_UST_ROOT is not set！")
        self.data_dir = os.path.join(
            LiBIM_UST_ROOT, cfg.seq
        )  # raw data
        self.seq_dir = os.path.join(
            LiBIM_UST_ROOT, "processed", cfg.seq, f"{cfg.interval:02d}m"
        )
        os.makedirs(self.seq_dir, exist_ok=True)
        self.submap3d_dir = os.path.join(
            self.data_dir,
            f"submap3d/{cfg.interval:02d}m/points",
        )
        self.submap3d_deprecated_dir = os.path.join(
            self.data_dir,
            f"submap3d_deprecated/{cfg.interval:02d}m/points",
        )
        self.kfmap3d_dir = os.path.join(
            self.data_dir,
            "kfmap3d/points",
        )

        self.wall_pcd_dir = os.path.join(
            self.data_dir,
            "frame/walls_points",
        )
        self.ground_pcd_dir = os.path.join(
            self.data_dir,
            "frame/ground_points",
        )
        self.kfmap3d_wall_pcd_dir = os.path.join(
            self.data_dir,
            "kfmap3d/walls_points",
        )
        self.kfmap3d_ground_pcd_dir = os.path.join(
            self.data_dir,
            "kfmap3d/ground_points",
        )

        self.submap3d_wall_pcd_dir = os.path.join(
            self.data_dir,
            f"submap3d/{cfg.interval:02d}m/walls_points",
        )
        self.submap3d_ground_pcd_dir = os.path.join(
            self.data_dir,
            f"submap3d/{cfg.interval:02d}m/ground_points",
        )
        self.submap3d_wall_pcd_deprecated_dir = os.path.join(
            self.data_dir,
            f"submap3d_deprecated/{cfg.interval:02d}m/walls_points",
        )
        self.submap3d_ground_pcd_deprecated_dir = os.path.join(
            self.data_dir,
            f"submap3d_deprecated/{cfg.interval:02d}m/ground_points",
        )
        self.occupied_pcd_dir = os.path.join(self.seq_dir, "occupied_pcd")
        self.occupied_pcd_flatten_dir = os.path.join(self.seq_dir, "occupied_pcd_flatten")
        self.free_pcd_dir = os.path.join(self.seq_dir, "free_pcd")
        self.lineseg_dir = os.path.join(self.seq_dir, "lineSeg")