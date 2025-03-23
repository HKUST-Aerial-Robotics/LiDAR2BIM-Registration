/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef POINT_SAM_OCTREE_SEG_H
#define POINT_SAM_OCTREE_SEG_H
#include "octree.h"
#include "utils/config.h"
#include <random>
#include "utils/bounding_box.h"
#include "travel/tgs.hpp"

class OctreeSeg{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;

    std::vector<int> instance_ids;
    Eigen::Vector3f cell_size;
    std::map<int, std::vector<OctreeMapNode::Ptr>> cell_indices;
	Octree::Ptr octree;

public:
    OctreeSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn){
		updateCloud(cloudIn);
    }

	void updateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn){
		cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloudIn, *cloud);
		instance_ids.resize(cloud->size(), -1);
		cell_size = Eigen::Vector3f(config::resolution[0], config::resolution[1], config::resolution[2]);
		octree.reset(new Octree(config::resolution));
	}

    void assignVoxelInstanceID(OctreeMapNode::Ptr cell, int instance_id){
        cell->instance_id = instance_id;
		for (auto &idx : cell->pointIdxList) {
			instance_ids[idx] = instance_id;
		}

        if (cell_indices.find(instance_id) == cell_indices.end()){
            cell_indices[instance_id] = std::vector<OctreeMapNode::Ptr>{cell};
        } else {
            cell_indices[instance_id].push_back(cell);
        }
    }

    void mergeInstance(OctreeMapNode& cell_erased, OctreeMapNode& cell_merged){
        int merged_id = cell_merged.instance_id;
        int erased_id = cell_erased.instance_id;
        for (auto &instance_id : instance_ids) {
            if (instance_id == erased_id) instance_id = merged_id;
        }
        for (auto &cell_key : cell_indices[erased_id]) {
			cell_key->instance_id = merged_id;
			for (auto &idx : cell_key->pointIdxList) {
				instance_ids[idx] = merged_id;
			}
        }
        cell_indices[merged_id].insert(cell_indices[merged_id].end(), cell_indices[erased_id].begin(), cell_indices[erased_id].end());
        cell_indices.erase(erased_id);
    }

	void segmentation(){
		TicToc t;
		// estimate ground
		travel::TravelGroundSeg<pcl::PointXYZ> travel_ground_seg;
		travel_ground_seg.setParams(200.0, 0.0);
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZ>);
		double time_taken;
		travel_ground_seg.estimateGround(*cloud, *ground, *nonground, time_taken);
		ground_cloud = ground;
		updateCloud(nonground);
//		LOG(INFO) << "estimate ground cost: " << t.toc() << " ms";

		octree->detectPlane(nonground);
//		LOG(INFO) << "detect plane cost: " << t.toc() << " ms" << ", points: " << nonground->size();
		octree->mergePlane();
//		LOG(INFO) << "merge plane cost: " << t.toc() << " ms";
		int global_index = 0;
		// extract planes at first
		for (auto it = octree->plane_map.begin(); it != octree->plane_map.end(); ++it) {
			auto& ins_vec = it->second;
			if (ins_vec.size() == 0) continue;
			for (auto& cur_cell : ins_vec) {
				if (!cur_cell->is_planar) continue;
				if (cur_cell->instance_id < 0){
					assignVoxelInstanceID(cur_cell, global_index);
					global_index++;
				}
				std::vector<VoxelKey> neighbors = getNeighbors(it->first);
				for (const VoxelKey &neighbor: neighbors) {
					auto neighbor_iter = octree->plane_map.find(neighbor);
					if (neighbor_iter == octree->plane_map.end()) continue;
					for (auto& neighbor_cell : neighbor_iter->second) {
						if (!neighbor_cell->is_planar) continue;
						if (cur_cell->samePlane(neighbor_cell)) {
							if (neighbor_cell->instance_id < 0) {
								assignVoxelInstanceID(neighbor_cell, cur_cell->instance_id);
							} else{
								if (neighbor_cell->instance_id == cur_cell->instance_id) continue;
								mergeInstance(*cur_cell, *neighbor_cell);
							}
						}
					}
				}
			}
		}
	
	
		// delete small instances
		for (auto it = cell_indices.begin(); it != cell_indices.end(); ++it) {
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_instance(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto& cell : it->second) {
				for (auto& idx : cell->pointIdxList) {
					cloud_instance->push_back(cloud->points[idx]);
				}
			}
			if (cloud_instance->size() < 3) {
				for (auto& cell : it->second) {
					for (auto& idx : cell->pointIdxList) {
						instance_ids[idx] = -1;
					}
				}
				continue;
			}
			const Eigen::Vector3d normal = it->second[0]->normal.cast<double>();
			BBoxUtils::RotatedRect rect = BBoxUtils::fittingBoundary<pcl::PointXYZ>(*cloud_instance, normal);
			
			if (rect.area < 0.5 || std::abs(normal.z()) > 0.35) {
				for (auto& instance_id : instance_ids) {
					if (instance_id == it->first) instance_id = -1;
				}
			}
			
		}
		
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
        pcl::copyPointCloud(*cloud, *cloudLabeledRGB);
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

	std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> getLabeledRGBInstances(){
		std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> instance_clouds;
		std::map<int, Color> instance_color_map = getSemanticColorMap();
		for (auto& id_color : instance_color_map){
			int instance_id = id_color.first;
			if (instance_id < 0) continue;
			auto& color = id_color.second;
			pcl::PointCloud<pcl::PointXYZRGBL>::Ptr instance_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
			for (int i = 0; i < cloud->size(); ++i) {
				auto& point = cloud->points[i];
				if (instance_ids[i] == instance_id){
					pcl::PointXYZRGBL point_rgb;
					point_rgb.x = point.x;
					point_rgb.y = point.y;
					point_rgb.z = point.z;
					point_rgb.r = std::get<0>(color);
					point_rgb.g = std::get<1>(color);
					point_rgb.b = std::get<2>(color);
					point_rgb.label = instance_id;
					instance_cloud->push_back(point_rgb);
				}
			}
			instance_clouds.push_back(instance_cloud);
		}
		return instance_clouds;
	}

    std::vector<int> getInstanceIDs(){
        return instance_ids;
    }

};

#endif //POINT_SAM_OCTREE_SEG_H
