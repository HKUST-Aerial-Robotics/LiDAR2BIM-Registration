#ifndef VOXEL_H
#define VOXEL_H

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <stdio.h>
#include <unordered_map>
#include <pcl/filters/voxel_grid.h>
#include "utils/geo_utils.h"

using VoxelKey = std::tuple<int, int, int>;
struct Vec3dHash {
    std::size_t operator()(const std::tuple<int, int, int> &vec3) const {
        return size_t(
                ((std::get<0>(vec3)) * 73856093) ^ ((std::get<1>(vec3)) * 471943) ^ ((std::get<2>(vec3)) * 83492791)) % 10000000;
    }
};

template<typename PointT>
VoxelKey hash_3d_key(const PointT &point, const Eigen::Vector3f &voxel_size) {
    int x = static_cast<int>(std::floor(point.x / voxel_size.x()));
    int y = static_cast<int>(std::floor(point.y / voxel_size.y()));
    int z = static_cast<int>(std::floor(point.z / voxel_size.z()));
    return std::make_tuple(x, y, z);
}

template<typename T>
VoxelKey hash_3d_key(const Eigen::Matrix<T, 3, 1> &point, const Eigen::Vector3f &voxel_size) {
    int x = static_cast<int>(std::floor(point.x() / voxel_size.x()));
    int y = static_cast<int>(std::floor(point.y() / voxel_size.y()));
    int z = static_cast<int>(std::floor(point.z() / voxel_size.z()));
    return std::make_tuple(x, y, z);
}

template<typename T>
Eigen::Matrix<T, 3, 1> key2center(const VoxelKey &key, const Eigen::Vector3f &voxel_size) {
    return Eigen::Matrix<T, 3, 1>(std::get<0>(key) * voxel_size.x(), std::get<1>(key) * voxel_size.y(),
                                  std::get<2>(key) * voxel_size.z()) + voxel_size / 2;
}

inline std::vector<VoxelKey> getNeighbors(const VoxelKey& loc){
	std::vector<VoxelKey> neighbors;
	neighbors.reserve(27); // 3^3
	int64_t x = std::get<0>(loc), y = std::get<1>(loc), z = std::get<2>(loc);
	for(int64_t i = x - 1; i <= x + 1; ++i){
		for(int64_t j = y - 1; j <= y + 1; ++j){
			for(int64_t k = z - 1; k <= z + 1; ++k){
				neighbors.emplace_back(i, j, k);
			}
		}
	}
	return neighbors;
}

class Voxel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Voxel> Ptr;

    Voxel(const VoxelKey k) {
        key = k;
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const {
        return cloud_;
    }

    std::vector<VoxelKey> neighbors(){
        return getNeighbors(key);
    }

    void insertPoint(const pcl::PointXYZ &point, int pt_id) {
        cloud_->push_back(point);
        point_ids.push_back(pt_id);
    }

    void check_planar(){
        Eigen::Vector4f plane_para;
        is_planar = fittingPlaneEigen<pcl::PointXYZ>(cloud_, plane_para);
        if (is_planar) {
            normal = Eigen::Vector3f(plane_para[0], plane_para[1], plane_para[2]);
            normal.normalize();
            center = cloud_->getMatrixXfMap().topRows(3).rowwise().mean().transpose();
        }
    }

    bool samePlane(const Voxel::Ptr& other){
        if(!is_planar || !other->is_planar){
            return false;
        }
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

public:
    int instance_id = -1;
    bool is_planar = false;
    std::vector<int> point_ids;
    VoxelKey key;
    Eigen::Vector3f center, normal;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

using VoxelMap = std::unordered_map<VoxelKey, Voxel::Ptr, Vec3dHash>;

template<typename PointT>
void cutCloud(const pcl::PointCloud<PointT> &cloud, const Eigen::Vector3f voxel_size, VoxelMap &voxel_map) {
    for (size_t i = 0; i < cloud.size(); i++) {
        VoxelKey position = hash_3d_key(cloud.points[i], voxel_size);
        VoxelMap::iterator voxel_iter = voxel_map.find(position);
        if (voxel_iter != voxel_map.end()) {
            voxel_iter->second->insertPoint(cloud.points[i], i);
        } else {
            Voxel::Ptr voxel = Voxel::Ptr(new Voxel(position));
            voxel->insertPoint(cloud.points[i], i);
            voxel_map.insert(std::make_pair(position, voxel));
        }
    }
}


#endif