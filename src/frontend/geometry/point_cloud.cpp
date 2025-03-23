#include "frontend/geometry/point_cloud.h"
#include <iostream>
#include <Eigen/Dense>
#include <memory>

PCD2d::PCD2d() : wallsPcd_(), groundPcd_(),
				 upperBound_(cfg::maxHeight), lowerBound_(cfg::minHeight),
				 voxelSize_(cfg::voxel_size),
				 T_rp_(Eigen::Matrix4d::Identity()),
				 T_rp_inv_(Eigen::Matrix4d::Identity()), ogWalls_(),
				 ogGround_(){};

PCD2d::PCD2d(
		pcl::PointCloud<pcl::PointXYZ>::Ptr groundPcd
) : wallsPcd_(), groundPcd_(groundPcd),
	upperBound_(cfg::maxHeight), lowerBound_(cfg::minHeight),
	voxelSize_(cfg::voxel_size),
	T_rp_(Eigen::Matrix4d::Identity()),
	T_rp_inv_(Eigen::Matrix4d::Identity()), ogWalls_(),
	ogGround_(groundPcd) {

//	pcl::VoxelGrid<PointT> voxelFilter;
//	voxelFilter.setInputCloud(groundPcd_);
//	voxelFilter.setLeafSize(1.0f, 1.0f, 1.0f); // 设置体素大小
//	voxelFilter.filter(*groundPcd_);
}

PCD2d::PCD2d(
		pcl::PointCloud<pcl::PointXYZ>::Ptr wallsPcd,
		pcl::PointCloud<pcl::PointXYZ>::Ptr groundPcd
) : wallsPcd_(wallsPcd), groundPcd_(groundPcd),
	upperBound_(cfg::maxHeight), lowerBound_(cfg::minHeight),
	voxelSize_(cfg::voxel_size), T_rp_(Eigen::Matrix4d::Identity()),
	T_rp_inv_(Eigen::Matrix4d::Identity()), ogWalls_(wallsPcd), ogGround_(groundPcd) {
	
	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setInputCloud(groundPcd_);
	voxelFilter.setLeafSize(1.0f, 1.0f, 1.0f); // 设置体素大小
	voxelFilter.filter(*groundPcd_);
}

bool PCD2d::GetPcd2d(const std::string &lidarOrigin) {
	if (!ResetRP(lidarOrigin)) return false;
//	CropWalls();
//	FlattenWalls();
	return true;
}

bool PCD2d::ResetRP(const std::string &lidarOrigin) {
//	auto downGround = groundPcd_->VoxelDownSample(1.0);
//	if (downGround->points_.size() < 3) {
//		std::cout << "Not enough points to fit the plane." << std::endl;
//		return false;
//	}
//	auto plane = downGround->SegmentPlane(0.1, 3, 20).first;
	timer.tic("Reset RP");
	
	
	if (groundPcd_->points.size() < 3) {
		std::cout << "Not enough points to fit the plane." << std::endl;
		return false;
	}
	pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.1);
	seg.setInputCloud(groundPcd_);
	seg.segment(*inliers, *planeCoefficients);
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return false;
	}
	Eigen::Vector4d plane;
	plane << planeCoefficients->values[0], planeCoefficients->values[1],
			planeCoefficients->values[2], planeCoefficients->values[3];
	Eigen::Vector3d t_l2w(0, 0, 0);
	
	t_l2w = ProjPt2Pl(t_l2w, plane);
	
	
	Eigen::Vector3d v1(0, 0, 1);
	Eigen::Vector3d v2(plane.head<3>());
	Eigen::Matrix3d R = RFrom2Vecs(v1, v2);
	Eigen::Vector3d t = -R.transpose() * t_l2w;
	
	T_rp_.block<3, 3>(0, 0) = R.transpose();
	T_rp_.block<3, 1>(0, 3) = t;
	
	T_rp_inv_ = T_rp_.inverse();
	//transform walls and ground
//	Eigen::Matrix4f T_rp_float = T_rp_.cast<float>();
//	pcl::transformPointCloud(*ogWalls_, *ogWalls_, T_rp_);
//	pcl::transformPointCloud(*ogGround_, *ogGround_, T_rp_);
	timer.toc("Reset RP");
	return true;
}

void PCD2d::CropWalls() {
	pcl::CropBox<PointT> cropBox = GetCropBox(GetBoundPcd(ogWalls_), upperBound_, lowerBound_);
	cropBox.setInputCloud(ogWalls_);
	cropBox.filter(*ogWallsCropped_);
}

void PCD2d::FlattenWalls() {
	pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>(*ogWallsCropped_));
	for (auto &point: tempCloud->points) {
		point.z = 0;
	}
	pcl::VoxelGrid<PointT> voxelGrid;
	voxelGrid.setInputCloud(tempCloud);
	voxelGrid.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
	voxelGrid.filter(*walls2d_);
}

Eigen::Matrix3d PCD2d::RFrom2Vecs(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) {
	Eigen::Vector3d v1Norm = v1.normalized();
	Eigen::Vector3d v2Norm = v2.normalized();
	Eigen::Vector3d axis = v1Norm.cross(v2Norm);
	double axisLength = axis.norm();
	double angle = acos(v1Norm.dot(v2Norm));
	return Eigen::AngleAxisd(angle, axis / axisLength).toRotationMatrix();
}


double PCD2d::GetPt2PlDist(const Eigen::Vector3d &pt, const Eigen::Vector4d &plane) {
	double a = plane[0], b = plane[1], c = plane[2], d = plane[3];
	return (a * pt[0] + b * pt[1] + c * pt[2] + d) / std::sqrt(a * a + b * b + c * c);
}

Eigen::Vector3d PCD2d::ProjPt2Pl(const Eigen::Vector3d &pt, const Eigen::Vector4d &plane) {
	double a = plane[0], b = plane[1], c = plane[2], d = plane[3];
	double x = pt[0], y = pt[1], z = pt[2];
	double pt2pl_dist = GetPt2PlDist(pt, plane);
	double norm = std::sqrt(a * a + b * b + c * c);
	return Eigen::Vector3d(
			x - a * pt2pl_dist / norm,
			y - b * pt2pl_dist / norm,
			z - c * pt2pl_dist / norm
	);
}

pcl::PointCloud<PointT>::Ptr PCD2d::GetBoundPcd(const pcl::PointCloud<PointT>::Ptr &pcd) {
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*pcd, minPt, maxPt);
	
	pcl::PointCloud<PointT>::Ptr boundPcd(new pcl::PointCloud<PointT>);
	boundPcd->emplace_back(minPt.x(), minPt.y(), minPt.z());
	boundPcd->emplace_back(maxPt.x(), minPt.y(), minPt.z());
	boundPcd->emplace_back(maxPt.x(), maxPt.y(), minPt.z());
	boundPcd->emplace_back(minPt.x(), maxPt.y(), minPt.z());
	
	return boundPcd;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> PCD2d::GetMinMax(const pcl::PointCloud<PointT>::Ptr &boundPcd) {
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*boundPcd, minPt, maxPt);
	
	Eigen::Vector3f minBound(minPt.x(), minPt.y(), 0.0f);
	Eigen::Vector3f maxBound(maxPt.x(), maxPt.y(), 0.0f);
	
	return {minBound, maxBound};
}

pcl::CropBox<PointT>
PCD2d::GetCropBox(const pcl::PointCloud<PointT>::Ptr &boundPcd, float upperBound, float lowerBound) {
	auto [minBound, maxBound] = GetMinMax(boundPcd);
	
	minBound.z() = lowerBound;
	maxBound.z() = upperBound;
	
	pcl::CropBox<PointT> cropBox;
	cropBox.setMin(Eigen::Vector4f(minBound.x(), minBound.y(), minBound.z(), 1.0));
	cropBox.setMax(Eigen::Vector4f(maxBound.x(), maxBound.y(), maxBound.z(), 1.0));
	
	return cropBox;
}