# Registration Benchmarks
We evaluate the performance of our method on ```LiBIM-UST``` datasets

[//]: # (+ Please update the YAML file by replacing the dataset_root variable with YOUR_PATH.)

[//]: # (  Run the following command to the evaluate a sequence of the dataset)

you can run this to evaluate a sequence:
```angular2html
export LiBIM_UST_ROOT="path/to/LiBIM-UST"
# e.g., export LiBIM_UST_ROOT="/home/bob/datasets/LiBIM-UST/"
bin/reg_bm benchmark /configs/interval/15m/2F/building_day.yaml
```
or you can run the bash script to evaluate all the sequences:
```angular2html
# remember to change the path of the LiBIM_UST_ROOT in the benchmark.sh
# for example, export LiBIM_UST_ROOT="/home/bob/datasets/LiBIM-UST/"
bash examples/scripts/benchmark.sh 
```

The LiBIM-UST dataset of this study has been upgraded to
the SLABIM dataset, available at https://github.com/HKUST-Aerial-Robotics/SLABIM. For more comprehensive evaluation, please refer to the SLABIM dataset. (The tutorial of the SLABIM dataset will be released soon.)