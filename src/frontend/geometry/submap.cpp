#include <vector>
#include <string>
#include <stdexcept>
#include <filesystem>
#include <pcl/visualization/pcl_visualizer.h>
#include "frontend/geometry/submap.h"

void SubmapManager::LoadSubmap() {
	//PCD
	std::ostringstream oss;
	oss << std::setw(6) << std::setfill('0') << index_;
	std::string reformatIndex = oss.str();
	std::string occupiedPcdFile = seqDir + "/occupied_pcd_flatten/" + reformatIndex + ".pcd";
	std::string freePcdFile = seqDir + "/free_pcd/" + reformatIndex + ".pcd";
	occupiedPcd_ = LoadPcd(occupiedPcdFile);
	freePcd_ = LoadPcd(freePcdFile);
	//lines
	std::string lineSegsFile = seqDir + "/lineSeg/" + reformatIndex + ".txt";
	lines_ = LoadLines(lineSegsFile);
	//GT
	std::string gtFile = seqDir + "/pose.txt";
	gt_ = LoadGT(gtFile);
	
}

std::vector<double> SubmapManager::LoadGT(std::string gtFile) {
	if (std::filesystem::exists(gtFile)) {
		std::ifstream file(gtFile);
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + gtFile);
		}
		
		std::string line;
		std::getline(file, line); // skip the first line
		
		std::vector<std::vector<double>> gtList;
		while (std::getline(file, line)) {
			std::istringstream iss(line);
			std::vector<double> values;
			double value;
			while (iss >> value) {
				values.push_back(value);
			}
			gtList.push_back(values);
		}
		
		if (index_ < 0 || index_ >= static_cast<int>(gtList.size())) {
			throw std::out_of_range("Index out of range");
		}
		
		return gtList[index_];
	} else {
		throw std::runtime_error("gt not found, copy gt to the " + gtFile);
	}
}

void SubmapManager::setT_rp() {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << index_;
    std::string reformatIndex = oss.str();

    std::string rawPcdFile = rawSeqDir + "/points/" + reformatIndex + ".pcd";
	rawPcd_ = LoadPcd(rawPcdFile);
    pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(rawPcd_);
	sor.setLeafSize(0.2f, 0.2f, 0.2f);
	sor.filter(*rawPcd_);

	std::string rawGroundPcdFile = rawSeqDir + "/ground_points/" + reformatIndex + ".pcd";
	rawgroundPcd_ = LoadPcd(rawGroundPcdFile);
	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setInputCloud(rawgroundPcd_);
	voxelFilter.setLeafSize(1.0f, 1.0f, 1.0f); // 设置体素大小
	voxelFilter.filter(*rawgroundPcd_);

	timer.tic("GetPcd2d");
	pcd2d_ = PCD2d(rawgroundPcd_);
	pcd2d_.GetPcd2d();
	timer.toc("GetPcd2d");
}
