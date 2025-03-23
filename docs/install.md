# Step-by-step installation instructions
**a. Install packages from ubuntu source.**
```shell
sudo apt-get update
sudo apt install libboost-dev libyaml-cpp-dev libomp-dev
# For backward-cpp
sudo apt-get install libgmp-dev libmpfr-dev
sudo apt install libpcl-dev pcl-tools
```
```shell
conda create -n libim python=3.10
conda activate libim
pip install -r requirements.txt
```
**b. Build Plane segmentation**
```commandline
mkdir -p Thirdparty/point-sam/build
cd Thirdparty/point-sam/build
cmake ..
make -j
cd ../../..
```


**c. Build**

The cpp implementation is in the ```src``` and ```include``` folder, and you can build it by running the following
command

```commandline
mkdir build
cd build
cmake ..
make -j
cd ..
```