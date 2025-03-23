#include "utils/utils.h"
#include <Eigen/SVD>
#include <cmath>

TicToc::TicToc() {}

void TicToc::tic(const std::string &keyName) {
    if (timeDict.find(keyName) == timeDict.end()) {
        timeDict[keyName] = TimerData();
    }
    timeDict[keyName].t0 = std::chrono::high_resolution_clock::now();
}

double TicToc::toc(const std::string &keyName, bool verbose) {
    if (timeDict.find(keyName) == timeDict.end()) {
        throw std::invalid_argument("No timer started for " + keyName);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsedTime = t1 - timeDict[keyName].t0;
    timeDict[keyName].time += elapsedTime.count();
    timeDict[keyName].number += 1;
    if (verbose) {
        std::cout << "Time for " << keyName << ": " << elapsedTime.count() << " ms" << std::endl;
    }
    return elapsedTime.count();
}

void TicToc::printSummary() const {
    // sort the keys
    std::vector<std::string> keys;
    for (const auto &[key, value]: timeDict) {
        keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    for (const auto &key: keys) {
        const auto &value = timeDict.at(key);
        double averageTime = value.number > 0 ? value.time / value.number : 0;
		LOG(INFO) << "Average Time for " << key << ": " << averageTime << " ms";
    }
}

double TicToc::getAvg(const std::string &keyName) const {
    if (timeDict.find(keyName) == timeDict.end()) {
        throw std::invalid_argument("No timer started for " + keyName);
    }
    return timeDict.at(keyName).time / timeDict.at(keyName).number;
}

void TicToc::clear() {
    timeDict.clear();
}

TicToc timer;

Stats::Stats() {}

void Stats::add(const std::string &keyName, double value) {
    statsDict[keyName].push_back(value);
}

std::vector<double> Stats::get(const std::string &keyName) const {
    if (statsDict.find(keyName) == statsDict.end()) {
        throw std::invalid_argument("No stats recorded for " + keyName);
    }
    return statsDict.at(keyName);
}

void Stats::printSummary() const {
    for (const auto &[key, values]: statsDict) {
        double avg = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        std::cout << "Stats for " << key << ": " << avg << std::endl;
    }
}

void Stats::clear() {
    statsDict.clear();
}

Eigen::Matrix2d svdRot2d(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y, const Eigen::VectorXd &W) {
    Eigen::VectorXd weights = W.size() == 0 ? Eigen::VectorXd::Ones(X.cols()) : W;
    Eigen::MatrixXd diagWeights = weights.asDiagonal();
    Eigen::MatrixXd H = X * diagWeights * Y.transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
        V.col(1) *= -1;
    }

    Eigen::Matrix2d rotationMatrix = V * U.transpose();
    return rotationMatrix;
}

Eigen::Vector3d svd2d(const Eigen::MatrixX2d &X, const Eigen::MatrixX2d &Y) {
	Eigen::RowVector2d Xmean = X.colwise().mean();
	Eigen::RowVector2d Ymean = Y.colwise().mean();
	Eigen::MatrixXd Xcentered = X.rowwise() - Xmean;
	Eigen::MatrixXd Ycentered = Y.rowwise() - Ymean;

	Eigen::MatrixXd H = Xcentered.transpose() * Ycentered;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXd U = svd.matrixU();
	Eigen::MatrixXd V = svd.matrixV();

	Eigen::Matrix2d R = V * U.transpose();
	if (R.determinant() < 0) {
		V.col(1) *= -1;
		R = V * U.transpose();
	}

	Eigen::RowVector2d t = Ymean - (R * Xmean.transpose()).transpose();
	double yaw = std::atan2(R(1, 0), R(0, 0));
	yaw = yaw * 180.0 / M_PI;
	return Eigen::Vector3d(t(0), t(1), yaw);
}

std::tuple<double, double, double>
solve2dClosedForm(const Eigen::Matrix<double, 3, 2> &source, const Eigen::Matrix<double, 3, 2> &target) {
    // Step 1: Center the points by subtracting their centroids
    Eigen::RowVector2d sourceCentroid = source.colwise().mean();
    Eigen::RowVector2d targetCentroid = target.colwise().mean();
    Eigen::MatrixXd sourceCentered = source.rowwise() - sourceCentroid;
    Eigen::MatrixXd targetCentered = target.rowwise() - targetCentroid;

    // Step 3: Compute the covariance matrix H
    Eigen::MatrixXd H = sourceCentered.transpose() * targetCentered;

    // Step 3: SVD of covariance matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Step 4: Compute the rotation matrix R
    Eigen::Matrix2d R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = V * U.transpose();
        return std::make_tuple(std::nan(""), std::nan(""), std::nan(""));
    }

    // Step 5: Compute the translation vector t
    Eigen::RowVector2d t = targetCentroid - (R * sourceCentroid.transpose()).transpose();

    // Step 6: Extract the rotation angle (yaw)
    double yaw = std::atan2(R(1, 0), R(0, 0));
    yaw = yaw * 180.0 / M_PI;

    return std::make_tuple(t(0), t(1), yaw);
}

double rotationToYaw(const Eigen::Matrix3d &rotation, bool degrees) {
    double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    if (degrees) {
        yaw = yaw * 180.0 / M_PI;
    }
    return yaw;
}

Eigen::Matrix4d xyyawToSe3(double x, double y, double yaw, bool degrees) {
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    if (degrees) {
        yaw = yaw * M_PI / 180.0;
    }
    se3(0, 0) = std::cos(yaw);
    se3(0, 1) = -std::sin(yaw);
    se3(1, 0) = std::sin(yaw);
    se3(1, 1) = std::cos(yaw);
    se3(0, 3) = x;
    se3(1, 3) = y;
    return se3;
}

std::vector<double> se32xyyaw(const Eigen::Matrix4d &se3, bool degrees) {
    double x = se3(0, 3);
    double y = se3(1, 3);
    double yaw = rotationToYaw(se3.block<3, 3>(0, 0), degrees);
    return {x, y, yaw};
}

Eigen::Matrix4d se32se2(const Eigen::Matrix4d &se3) {
    double yaw = rotationToYaw(se3.block<3, 3>(0, 0), true);
    Eigen::Matrix4d se2 = Eigen::Matrix4d::Identity();
    se2(0, 0) = std::cos(yaw * M_PI / 180.0);
    se2(0, 1) = -std::sin(yaw * M_PI / 180.0);
    se2(1, 0) = std::sin(yaw * M_PI / 180.0);
    se2(1, 1) = std::cos(yaw * M_PI / 180.0);
    se2(0, 3) = se3(0, 3);
    se2(1, 3) = se3(1, 3);
    return se2;
}

void PointCloudToEigen(const pcl::PointCloud<PointT>::Ptr &cloud, Eigen::MatrixX2d &matrix) {
    matrix.resize(cloud->points.size(), 2);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        matrix(i, 0) = cloud->points[i].x;
        matrix(i, 1) = cloud->points[i].y;
    }
}

void PointCloudToEigen(const pcl::PointCloud<PointT>::Ptr &cloud, std::vector<Eigen::Vector2d> &points) {
    points.resize(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        points[i] = Eigen::Vector2d(cloud->points[i].x, cloud->points[i].y);
    }
}