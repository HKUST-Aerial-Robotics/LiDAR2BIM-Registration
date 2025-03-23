/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_VOXEL_CLUSTERING_H
#define SRC_VOXEL_CLUSTERING_H
#include "voxel.h"
#include <random>

class VoxelClustering{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<int> instance_ids;
    Eigen::Vector3f voxel_size;
    VoxelMap voxelMap;
    std::map<int, std::vector<VoxelKey>> voxel_indices;

public:
    VoxelClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn){
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloudIn, *cloud);
        instance_ids.resize(cloud->size(), -1);
        voxel_size = Eigen::Vector3f(0.1, 0.1, 0.1);
    }

    void assignVoxelInstanceID(Voxel& voxel, int instance_id){
        voxel.instance_id = instance_id;
        for (int i = 0; i < voxel.point_ids.size(); ++i) {
            instance_ids[voxel.point_ids[i]] = instance_id;
        }
        if (voxel_indices.find(instance_id) == voxel_indices.end()){
            voxel_indices[instance_id] = std::vector<VoxelKey>{voxel.key};
        } else {
            voxel_indices[instance_id].push_back(voxel.key);
        }
    }

    void mergeInstance(Voxel& voxel_erased, Voxel& voxel_merged){
        int merged_id = voxel_merged.instance_id;
        int erased_id = voxel_erased.instance_id;
        for (auto &instance_id : instance_ids) {
            if (instance_id == erased_id) instance_id = merged_id;
        }
        for (auto &voxel_key : voxel_indices[erased_id]) {
            voxelMap[voxel_key]->instance_id = merged_id;
        }
        voxel_indices[merged_id].insert(voxel_indices[merged_id].end(), voxel_indices[erased_id].begin(), voxel_indices[erased_id].end());
        voxel_indices.erase(erased_id);
    }

    void clustering(){
        cutCloud(*cloud, voxel_size, voxelMap);
        for (auto it = voxelMap.begin(); it != voxelMap.end(); ++it) {
            it->second->instance_id = -1;
        }

        int global_index = 0;
        for (auto it = voxelMap.begin(); it != voxelMap.end(); ++it) {
            auto& cur_voxel = it->second;
            if (cur_voxel->instance_id < 0){
                assignVoxelInstanceID(*cur_voxel, global_index);
                global_index++;
            }

            const std::vector<VoxelKey>& neighbors = cur_voxel->neighbors();
            for (const VoxelKey &neighbor: neighbors) {
                auto neighbor_iter = voxelMap.find(neighbor);
                if (neighbor_iter == voxelMap.end()) continue;
                Voxel& neighbor_voxel = *neighbor_iter->second;
                if (neighbor_voxel.instance_id < 0) {
                    assignVoxelInstanceID(neighbor_voxel, cur_voxel->instance_id);
                } else{
                    if (neighbor_voxel.instance_id == cur_voxel->instance_id) continue;
                    mergeInstance(*cur_voxel, neighbor_voxel);
                }
            }
        }
    }

    std::vector<int> getInstanceIDs(){
        return instance_ids;
    }

};

class VoxelClusterSemantic{
public:
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudLabeled;
    std::vector<int> instance_ids;

public:

    VoxelClusterSemantic(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloudIn){
        cloudLabeled.reset(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::copyPointCloud(*cloudIn, *cloudLabeled);
        instance_ids.resize(cloudLabeled->size(), -1);
    }

    VoxelClusterSemantic(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloudIn){
        cloudLabeled.reset(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::copyPointCloud(*cloudIn, *cloudLabeled);
        instance_ids.resize(cloudLabeled->size(), -1);
    }

    void clustering(){
        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> semantic_cloud_map; // semantic_id -> cloud
        std::map<int, std::map<int, int>> semantic_map; // semantic_id -> local_id -> global_id
        for (int i = 0; i < cloudLabeled->size(); ++i) {
            auto& point = cloudLabeled->points[i];
            int semantic_id = point.label;
            if (semantic_id == 0) continue;
            if (semantic_cloud_map.find(semantic_id) == semantic_cloud_map.end()){
                semantic_cloud_map[semantic_id].reset(new pcl::PointCloud<pcl::PointXYZ>);
                semantic_map[semantic_id] = std::map<int, int>();
            }
            pcl::PointXYZ point_xyz;
            point_xyz.x = point.x;
            point_xyz.y = point.y;
            point_xyz.z = point.z;
            semantic_cloud_map[semantic_id]->push_back(point_xyz);
            int local_id = semantic_cloud_map[semantic_id]->size() - 1;
            semantic_map[semantic_id][local_id] = i;
        }

        for (auto it = semantic_cloud_map.begin(); it != semantic_cloud_map.end(); ++it) {
            auto& semantic_cloud = it->second;
            int semantic_id = it->first;

            VoxelClustering voxelClustering(semantic_cloud);
            voxelClustering.clustering();
            const std::vector<int>& instance_ids = voxelClustering.getInstanceIDs();
            for (int i = 0; i < instance_ids.size(); ++i) {
                int global_id = semantic_map[semantic_id][i];
                this->instance_ids[global_id] = instance_ids[i] + semantic_id * 1000;
            }
        }
    }

    std::vector<int> getInstanceIDs(){
        return instance_ids;
    }

    // Define a color as a tuple of three ints
    typedef std::tuple<int, int, int> Color;
    std::map<int, Color> getSemanticColorMap(){
        std::map<int, Color> instance_color_map;
        std::set<int> ins_id_set = std::set<int>(instance_ids.begin(), instance_ids.end());
        instance_color_map[-1] = Color(0, 0, 0);
        for (int id: ins_id_set) {
            if (id < 0) continue;
            std::mt19937 rng(std::random_device{}());
            std::uniform_int_distribution<int> dist(0, 255);
            int r = dist(rng);
            int g = dist(rng);
            int b = dist(rng);
            instance_color_map[id] = Color(r, g, b);
        }
        return instance_color_map;
    }

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr getLabeledRGBCloud(){
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloudLabeledRGB(new pcl::PointCloud<pcl::PointXYZRGBL>);
        pcl::copyPointCloud(*cloudLabeled, *cloudLabeledRGB);
        std::map<int, Color> instance_color_map = getSemanticColorMap();
        for (int i = 0; i < cloudLabeledRGB->size(); ++i) {
            auto& point = cloudLabeledRGB->points[i];
            int instance_id = instance_ids[i];
            if (instance_color_map.find(instance_id) == instance_color_map.end()) continue;
            auto& color = instance_color_map[instance_id];
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
            point.label = std::max(0, instance_id);
        }
        return cloudLabeledRGB;
    }
};



#endif //SRC_VOXEL_CLUSTERING_H
