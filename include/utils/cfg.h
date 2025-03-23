/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_CONFIG_H
#define BIMREG_CONFIG_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <glog/logging.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;

namespace cfg {

    extern std::string project_path, config_file, seq, floor;
    extern double max_side_length, lengthRes, angleRes, xyRes, yawRes, nmsDist, gridSize, sdfDist, sigma, freeErodeWidth, lamda, maxHeight, minHeight, voxel_size;
    extern int interval, topL, topK, topJ, NeighThreshold;
	extern bool augmented;

    template<typename T>
    T get(const YAML::Node &node, const std::string &key, const T &default_value) {
        if (!node[key]) {
            LOG(INFO) << "Key " << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[key].as<T>();
        LOG(INFO) << "Key " << key << " found, using value: " << value;
        return value;
    }

    template<typename T>
    T get(const YAML::Node &node, const std::string &father_key, const std::string &key, const T &default_value) {
        if (!node[father_key] || !node[father_key][key]) {
            LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[father_key][key].as<T>();
        LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << value;
        return value;
    }

    void readParameters(std::string config_file_);
}

void InitGLOG(std::string config_path);

#endif //BIMREG_CONFIG_H
