import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.parent))
from preprocess.datasets.libim_ust_utils import LiBIMUSTDataset
from preprocess.config import load_cfg


def make_all_benchmarks(config_list):
    for config_path in config_list:
        cfg = load_cfg(config_path)
        # Load dataset
        dataset = LiBIMUSTDataset(cfg)
        dataset.make_benchmarks(clear=True, refine=True)
    config_path = "./configs/interval/15m/2F/office.yaml"
    cfg = load_cfg(config_path)
    dataset = LiBIMUSTDataset(cfg)
    dataset.merge_office()

    config_path = "./configs/interval/30m/2F/office.yaml"
    cfg = load_cfg(config_path)
    dataset = LiBIMUSTDataset(cfg)
    dataset.merge_office()

def check_pose(config_path):
    cfg = load_cfg(config_path)
    # Load dataset
    dataset = LiBIMUSTDataset(cfg)
    dataset.check_pose(0)


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
    print("Steps2/2: Making benchmarks...")
    make_all_benchmarks(config_list)
    print("Prepration Done！！")
