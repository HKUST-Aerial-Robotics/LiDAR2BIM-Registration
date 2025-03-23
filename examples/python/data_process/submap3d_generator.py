import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.parent))
from preprocess.datasets.libim_ust_utils import LiBIMUSTDataset
from preprocess.config import load_cfg


def generate_all_submap3d(config_list):
    for config_path in config_list:
        print(f"Generating submaps for {config_path}...")
        cfg = load_cfg(config_path)
        # Load dataset
        dataset = LiBIMUSTDataset(cfg)
        dataset.generate_submap3d(viz=False, save=True, clear=True,deprecated=False)  # Generate submap3d only
    print("Done, please use pointsam to segment the submap3d")


if __name__ == "__main__":
    config_list = [
        "./configs/interval/15m/2F/building_day.yaml",
        "./configs/interval/15m/2F/2f_office_01.yaml",
        "./configs/interval/15m/2F/2f_office_02.yaml",
        "./configs/interval/15m/2F/2f_office_03.yaml",
        "./configs/interval/30m/2F/2f_office_01.yaml",
        "./configs/interval/30m/2F/2f_office_02.yaml",
        "./configs/interval/30m/2F/2f_office_03.yaml",
    ]
    print("Steps1/2: Generating submaps...")
    generate_all_submap3d(config_list)
