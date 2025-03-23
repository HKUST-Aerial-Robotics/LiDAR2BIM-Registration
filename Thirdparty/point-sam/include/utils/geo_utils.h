/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef POINT_SAM_GEO_UTILS_H
#define POINT_SAM_GEO_UTILS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "utils/config.h"
#include "utils/tic_toc.h"
template<typename PointT, typename T>
void solveCovMat(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<T, 3, 1> &mu,
                 Eigen::Matrix<T, 3, 3> &cov) {
    mu.setZero();
    cov.setZero();
    Eigen::Matrix<T, 3, 1> point;
    auto N = cloud.size();
    for (int i = 0; i < N; ++i) {
        point = cloud.points[i].getVector3fMap().template cast<T>();
        mu += point;
        cov += point * point.transpose();
    }
    mu /= N;
    cov.noalias() = cov / N - mu * mu.transpose();
}

template<typename PointT, typename T>
void solveWeightedCovMat(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<T, 3, 1> &mu,
				 Eigen::Matrix<T, 3, 3> &cov) {
	mu.setZero();
	auto N = cloud.size();
	for (int i = 0; i < N; ++i) {
		mu += cloud.points[i].getVector3fMap().template cast<T>();
	}
	mu /= N;

	std::vector<T> weights(N, 0);
	for (int i = 0; i < N; ++i) {
		weights[i] = 1 - (cloud.points[i].getVector3fMap().template cast<T>() - mu).norm();
	}
	T sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	for (int i = 0; i < N; ++i) {
		weights[i] /= sum;
	}
	cov.setZero();
	for (int i = 0; i < N; ++i) {
		auto diff = cloud.points[i].getVector3fMap().template cast<T>() - mu;
		cov += weights[i] * diff * diff.transpose();
	}
}

template<typename PointT, typename T>
void solveCenter(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<T, 3, 1> &mu) {
    mu.setZero();
    Eigen::Matrix<T, 3, 1> point;
    auto N = cloud.size();
    for (int i = 0; i < N; ++i) {
        point = cloud.points[i].getVector3fMap().template cast<T>();
        mu += point;
    }
    mu /= N;
}

template<typename PointT>
bool fittingPlaneEigen(const typename pcl::PointCloud<PointT>::Ptr &cloud, Eigen::Vector4f &plane){

	if (cloud->size() <= 3) {
		return false;
	}

	Eigen::Matrix3f cov;
	Eigen::Vector3f mu;
	solveCovMat(*cloud, mu, cov);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
	if (eigensolver.info() != Eigen::Success) {
		return false;
	}
	Eigen::Vector3f normal = eigensolver.eigenvectors().col(0);
	normal.normalize();
	plane << normal(0), normal(1), normal(2), -normal.dot(mu);

	Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();
	float planarity = eigenvalues(1) / eigenvalues(0);
	if (planarity > config::eigen_thd) {
		return true;
	} else {
		return false;
	}
}

template<typename PointT>
bool fittingPlaneRANSAC(const typename pcl::PointCloud<PointT>::Ptr &cloud, Eigen::Vector4f &plane){

	if (cloud->size() <= 3) {
		return false;
	}

	TicToc t_ransac;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (config::plane_ransac_iter);
    seg.setDistanceThreshold (config::plane_ransac_thd);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    plane << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];

    bool is_planar;
    if (inliers->indices.size() >= 0.9 * cloud->size()) {
		// only keep the inliers
		pcl::copyPointCloud(*cloud, inliers->indices, *cloud);
        is_planar = true;
    } else {
        is_planar = false;
    }
    return is_planar;
}

template<typename PointT>
void visualizeCloud(typename pcl::PointCloud<PointT>::Ptr cloud, std::string name) {
	pcl::visualization::PCLVisualizer viewer(name);
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud<PointT>(cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	// set camera position at the center of the point cloud
	viewer.setCameraPosition(cloud->points[cloud->size() / 2].x,
							 cloud->points[cloud->size() / 2].y,
							 cloud->points[cloud->size() / 2].z,
							 0, 0, 1);
	viewer.spin();
	viewer.close();
}

template<typename PointT>
void visualizeCloud(typename pcl::PointCloud<PointT>::Ptr cloudA, typename pcl::PointCloud<PointT>::Ptr cloudB, std::string name) {
	pcl::visualization::PCLVisualizer viewer(name);
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud<PointT>(cloudA, "cloudA", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloudA");
	viewer.addPointCloud<PointT>(cloudB, "cloudB", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloudB");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	// set camera position at the center of the point cloud
	viewer.setCameraPosition(cloudA->points[cloudA->size() / 2].x,
							 cloudA->points[cloudA->size() / 2].y,
							 cloudA->points[cloudA->size() / 2].z,
							 0, 0, 1);
	viewer.spin();
	viewer.close();
}

#endif //POINT_SAM_GEO_UTILS_H
