/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_UTILS_H
#define BIMREG_UTILS_H

#include <chrono>
#include <iostream>
#include <cstdlib>
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <tuple>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "cfg.h"

class TicToc {
public:
    TicToc();

    void tic(const std::string &keyName);

    double toc(const std::string &keyName, bool verbose = false);

    void printSummary() const;

    double getAvg(const std::string &keyName) const;

    void clear();

private:
    struct TimerData {
        double time = 0;
        int number = 0;
        std::chrono::high_resolution_clock::time_point t0;
    };

    std::unordered_map<std::string, TimerData> timeDict;
};

extern TicToc timer;

class Stats {
public:
    Stats();

    void add(const std::string &keyName, double value);

    std::vector<double> get(const std::string &keyName) const;

    void printSummary() const;

    void clear();

private:
    std::unordered_map<std::string, std::vector<double>> statsDict;
};

Eigen::Matrix2d
svdRot2d(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y, const Eigen::VectorXd &W = Eigen::VectorXd());

std::tuple<double, double, double>
solve2dClosedForm(const Eigen::Matrix<double, 3, 2> &source, const Eigen::Matrix<double, 3, 2> &target);

Eigen::Vector3d
svd2d(const Eigen::MatrixX2d &X, const Eigen::MatrixX2d &Y);

double rotationToYaw(const Eigen::Matrix3d &rotation, bool degrees = false);

Eigen::Matrix4d xyyawToSe3(double x, double y, double yaw, bool degrees = false);

std::vector<double> se32xyyaw(const Eigen::Matrix4d &se3, bool degrees = false);

Eigen::Matrix4d se32se2(const Eigen::Matrix4d &se3);

void PointCloudToEigen(const pcl::PointCloud<PointT>::Ptr &cloud, Eigen::MatrixX2d &matrix);

void PointCloudToEigen(const pcl::PointCloud<PointT>::Ptr &cloud, std::vector<Eigen::Vector2d> &points);

inline std::string getROOT(){
    const char* env_path = std::getenv("LiBIM_UST_ROOT");
    if (env_path != nullptr) {
        std::string path(env_path);
        return path;
    } else {
        std::cerr << "LiBIM_UST_ROOT is not set！" << std::endl;
        exit(1);
    }
}

#endif //BIMREG_UTILS_H