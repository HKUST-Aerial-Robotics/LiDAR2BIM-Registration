## Dataset

you can use prepare_data.sh to preprocess the ```LiBIM-UST``` dataset to test, and here is
the link
of [LiBIM-UST](https://hkustconnect-my.sharepoint.com/:u:/g/personal/hhuangce_connect_ust_hk/ERoCJi5Q5EZGudr8pQYOXsMBjUsin82b0RYxDVmVQmYCDg?e=ecnrwF)

Here is the folder structure of original ```LiBIM-UST```
```angular2html
LiBIM-UST
├── 2f_office_01
│   ├── frame
│   │   └── points
│   ├── pose
│   └── station
├── 2f_office_02
│   ├── frame
│   │   └── points
│   ├── pose
│   └── station
├── 2f_office_03
│   ├── frame
│   │   └── points
│   ├── pose
│   └── station
├── BIM
│   └── 2F
└── BuildingDay
    ├── frame
    │   └── points
    ├── pose
    └── station
```

Then you can run the following command to process the dataset

```bash
# remember to change the path of the LiBIM_UST_ROOT to your own path
export LiBIM_UST_ROOT="/media/qzj/disk2T-a/LiBIM-UST/"

# Steps 1/2: Generate the submaps according to the trajectory for registration, and segment the submaps into free space and occupied space
python examples/python/data_process/submap3d_generator.py
./Thirdparty/point-sam/build/plane_seg

# Steps 2/2: make benchmark data for registration, including the line segment and the ground truth pose.
python examples/python/data_process/make_benchmarks.py
```

and you will get the preprocessed ```LiBIM-UST``` dataset.


Here is the folder structure of processed ```LiBIM-UST```
```angular2html
LiBIM-UST
├── 2f_office_01
│   ├── frame
│   │   ├── ground_points
│   │   ├── points
│   │   └── walls_points
│   ├── pose
│   ├── station
│   └── submap3d
│       ├── 15m
│       │   ├── ground_points
│       │   ├── points
│       │   └── walls_points
│       └── 30m
│           ├── ground_points
│           ├── points
│           └── walls_points
├── 2f_office_02
│   ├── frame
│   │   ├── ground_points
│   │   ├── points
│   │   └── walls_points
│   ├── pose
│   ├── station
│   └── submap3d
│       ├── 15m
│       │   ├── ground_points
│       │   ├── points
│       │   └── walls_points
│       └── 30m
│           ├── ground_points
│           ├── points
│           └── walls_points
├── 2f_office_03
│   ├── frame
│   │   ├── ground_points
│   │   ├── points
│   │   └── walls_points
│   ├── pose
│   ├── station
│   └── submap3d
│       ├── 15m
│       │   ├── ground_points
│       │   ├── points
│       │   └── walls_points
│       └── 30m
│           ├── ground_points
│           ├── points
│           └── walls_points
├── BIM
│   └── 2F
├── BuildingDay
│   ├── frame
│   │   ├── ground_points
│   │   ├── points
│   │   └── walls_points
│   ├── pose
│   ├── station
│   └── submap3d
│       └── 15m
│           ├── ground_points
│           ├── points
│           └── walls_points
└── processed
├── 2f_office_01
│   ├── 15m
│   │   ├── free_pcd
│   │   ├── lineSeg
│   │   └── occupied_pcd_flatten
│   └── 30m
│       ├── free_pcd
│       ├── lineSeg
│       └── occupied_pcd_flatten
├── 2f_office_02
│   ├── 15m
│   │   ├── free_pcd
│   │   ├── lineSeg
│   │   └── occupied_pcd_flatten
│   └── 30m
│       ├── free_pcd
│       ├── lineSeg
│       └── occupied_pcd_flatten
├── 2f_office_03
│   ├── 15m
│   │   ├── free_pcd
│   │   ├── lineSeg
│   │   └── occupied_pcd_flatten
│   └── 30m
│       ├── free_pcd
│       ├── lineSeg
│       └── occupied_pcd_flatten
├── BuildingDay
│   └── 15m
│       ├── free_pcd
│       ├── lineSeg
│       └── occupied_pcd_flatten
└── Office
    ├── 15m
    │   ├── free_pcd
    │   ├── lineSeg
    │   └── occupied_pcd_flatten
    └── 30m
    ├── free_pcd
    ├── lineSeg
    └── occupied_pcd_flatten


```

It's noticeable that different rounds of preprocessing of the dataset could provide different recall rates of
registration(~ 1 to 2 percent fluctuation) because of some randomness in the linesegment extraction.

If you do not want to wait for a long time to process the dataset, you can download the ```processed``` folder from the following
link [processed](https://hkustconnect-my.sharepoint.com/:u:/g/personal/hhuangce_connect_ust_hk/EYiHE-lKVwdLgLRxfDWMpTwBok3Dk_OjmkSkhejte-FcfA?e=AljpIr)， and simply put it in the ```LiBIM-UST``` folder.
Then you can run the evaluation in the [Benchmarks](benchmark.md). 

However, to run the [Demo](demo.md), you still need to run the Steps 1/2

