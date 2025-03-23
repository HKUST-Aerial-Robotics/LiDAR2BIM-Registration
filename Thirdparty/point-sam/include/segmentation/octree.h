#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <stdio.h>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/shared_ptr.hpp>
#include "voxel.h"

class OctreeMapNode {

public:
	using Ptr = std::shared_ptr<OctreeMapNode>;
	OctreeMapNode::Ptr ChildNodes[8];
	bool is_planar, isLeafNode;
	Eigen::Vector3f centroid, normal, voxel_size, center;
	int level;
	int instance_id = -1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud; // cloud memory
	std::list<int> pointIdxList;

	OctreeMapNode(int l, Eigen::Vector3f ctr, pcl::PointCloud<pcl::PointXYZ>::Ptr ptr) : level(l), centroid(ctr), pCloud(ptr) {
		voxel_size = Eigen::Vector3f(config::resolution[0] / pow(2, level), config::resolution[1] / pow(2, level), config::resolution[2] / pow(2, level));
		is_planar = false;
		isLeafNode = level + 1 >= config::oc_depth ? true : false;
		for (int i = 0; i < 8; i++)
			this->ChildNodes[i] = nullptr;
	}

	void insert(const int& idx) {
		pointIdxList.push_back(idx);
	}

	void setPlaneNode(const Eigen::Vector4f& plane_para){
		normal = plane_para.head<3>();
		center = Eigen::Vector3f(0, 0, 0);
		for (auto& idx : pointIdxList) {
			center += pCloud->points[idx].getVector3fMap();
		}
		center /= pointIdxList.size();
		is_planar = true;
		isLeafNode = true;
	}
	void deleteUnfittedPoint()
	{
		for (auto it = pointIdxList.begin(); it != pointIdxList.end();)
		{
			const pcl::PointXYZ &point = pCloud->points[*it];
			double dist = std::abs((point.getVector3fMap() - center).dot(normal));
			
			if (dist > config::plane_ransac_thd)
			{	
				it = pointIdxList.erase(it); // erase 返回新的有效迭代器
			}
			else
			{
				++it; // 只在不删除元素时递增迭代器
			}
		}
	}

	int returnChildIdx(const pcl::PointXYZ &point){
		int idx = 0;
		if (point.x > centroid[0]) idx |= 4;
		if (point.y > centroid[1]) idx |= 2;
		if (point.z > centroid[2]) idx |= 1;
		return idx;
	}

	Eigen::Vector3f getChildCtr(int idx){
		Eigen::Vector3f ctr = centroid;
		if (idx & 4) ctr[0] += voxel_size[0] / 2;
		else ctr[0] -= voxel_size[0] / 2;
		if (idx & 2) ctr[1] += voxel_size[1] / 2;
		else ctr[1] -= voxel_size[1] / 2;
		if (idx & 1) ctr[2] += voxel_size[2] / 2;
		else ctr[2] -= voxel_size[2] / 2;
		return ctr;
	}

	void split(){
		if (pointIdxList.size() == 0) return;
		if (isLeafNode) return;
		for (auto& idx : pointIdxList) {
			const pcl::PointXYZ& point = pCloud->points[idx];
			int oc_idx = returnChildIdx(point);
			if (ChildNodes[oc_idx] == nullptr) {
				ChildNodes[oc_idx] = OctreeMapNode::Ptr(new OctreeMapNode(level + 1, getChildCtr(oc_idx), pCloud));
			}
			ChildNodes[oc_idx]->insert(idx);
		}
	}

	void detectPlane(){
		Eigen::Vector4f plane_para;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
		for (auto& idx : pointIdxList) {
			cloudTmp->points.push_back(pCloud->points[idx]);
		}
		bool is_planar = fittingPlaneEigen<pcl::PointXYZ>(cloudTmp, plane_para);
		if (is_planar) {
			setPlaneNode(plane_para);
			deleteUnfittedPoint();
			if (pointIdxList.size() <= 3) {
				std::cout << "empty node" << std::endl;
				std::cout << "pointIdxList.size(): " << pointIdxList.size() << std::endl;
				is_planar = false;
			}
		} else if (!isLeafNode){
			split();
			for (int i = 0; i < 8; i++) {
				if (ChildNodes[i] != nullptr) {
					ChildNodes[i]->detectPlane();
				}
			}
		} else {
			is_planar = false;
		}
	}

	bool samePlane(const OctreeMapNode::Ptr& other){
		if(!is_planar || !other->is_planar){
			return false;
		}
		double d1 = std::abs((center - other->center).dot(normal));
		double d2 = std::abs((center - other->center).dot(other->normal));
//		LOG(INFO) << "center: " << center.transpose() << " other center: " << other->center.transpose();
//		LOG(INFO) << "normal: " << normal.transpose() << " other normal: " << other->normal.transpose();
//		LOG(INFO) << "d1: " << d1 << " d2: " << d2 << " plane_merge_thd_p: " << config::plane_merge_thd_p;
		// point to plane distance
		if(std::abs((center - other->center).dot(normal)) > config::plane_merge_thd_p){
			return false;
		}
		if(std::abs((center - other->center).dot(other->normal)) > config::plane_merge_thd_p){
			return false;
		}
		if(std::abs(normal.dot(other->normal)) < config::plane_merge_thd_n){
			return false;
		}
		return true;
	}

	void merge(const OctreeMapNode& other){
		for (auto& idx : other.pointIdxList) {
			pointIdxList.push_back(idx);
		}
	}

	void mergePlane(std::vector<OctreeMapNode::Ptr>& instances){
		if (is_planar){
			bool is_merged = false;
			for (int i = 0; i < instances.size(); i++){
				if (samePlane(instances[i])){
					instances[i]->merge(*this);
					is_merged = true;
					break;
				}
			}
			if (!is_merged){
				this->instance_id = -1;
				OctreeMapNode::Ptr instance = OctreeMapNode::Ptr(new OctreeMapNode(*this));
				instances.push_back(instance);
			}
		} else {
			for (int i = 0; i < 8; i++) {
				if (ChildNodes[i] != nullptr) {
					ChildNodes[i]->mergePlane(instances);
				}
			}
		}
	}

	// copy constructor
	OctreeMapNode(const OctreeMapNode& other){
		level = other.level;
		centroid = other.centroid;
		voxel_size = other.voxel_size;
		normal = other.normal;
		center = other.center;
		is_planar = other.is_planar;
		isLeafNode = other.isLeafNode;
		instance_id = other.instance_id;
		pCloud = other.pCloud;
		pointIdxList = other.pointIdxList;
		for (int i = 0; i < 8; i++) {
			if (other.ChildNodes[i] != nullptr) {
				ChildNodes[i] = OctreeMapNode::Ptr(new OctreeMapNode(*other.ChildNodes[i]));
			}
		}
	}

};

class Octree {
public:
    using Ptr = std::shared_ptr<Octree>;
	std::unordered_map<VoxelKey, OctreeMapNode::Ptr, Vec3dHash> oc_map;
	using PlaneMap = std::unordered_map<VoxelKey, std::vector<OctreeMapNode::Ptr>, Vec3dHash>;
	PlaneMap plane_map;

private:
	Eigen::Vector3f voxel_size_;

public:
    Octree(Eigen::Vector3f voxel_size){
        voxel_size_ = voxel_size;
    }

    ~Octree() {
		oc_map.clear();
    }

    void detectPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pCloud) {
		TicToc t;

		for (int idx = 0; idx < pCloud->points.size(); idx++) {
			const pcl::PointXYZ& point = pCloud->points[idx];
			VoxelKey key = hash_3d_key(point, voxel_size_);
			if (oc_map.find(key) == oc_map.end()) {
				Eigen::Vector3f centroid = key2center<float>(key, voxel_size_);
				oc_map[key] = OctreeMapNode::Ptr(new OctreeMapNode(0, centroid, pCloud));
			}
			oc_map[key]->insert(idx);
		}

		for (auto it = oc_map.begin(); it != oc_map.end(); it++) {
			OctreeMapNode::Ptr node = it->second;
			node->detectPlane();
		}
    }

	PlaneMap mergePlane(){
		for (auto it = oc_map.begin(); it != oc_map.end(); it++) {
			std::vector<OctreeMapNode::Ptr> instances;
			it->second->mergePlane(instances);

			if (instances.size() > 0){
				plane_map[it->first] = instances;
			}
		}
		return plane_map;
	}
};


#endif