#include "utils/utils.h"
#include "utils/cfg.h"
#include "utils/analysis.h"
#include "frontend/geometry/corner.h"
#include "frontend/geometry/bim.h"
#include "frontend/geometry/submap.h"
#include "backend/reglib.h"
#include "global_definition/global_definition.h"
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "utils/cloud_dist.h"
#include "utils/tqdm.hpp"

namespace fs = std::filesystem;
bool vizClusters = false;
bool analyzeVerification = false;

int countFilesInDirectory(const fs::path &directory) {
	int file_count = 0;
	if (fs::exists(directory) && fs::is_directory(directory)) {
		for (const auto &entry: fs::directory_iterator(directory)) {
			if (fs::is_regular_file(entry.status())) {
				++file_count;
			}
		}
	} else {
		std::cerr << "Directory does not exist: " << directory << std::endl;
	}
	return file_count;
}

std::vector<double>
calculateScore(std::shared_ptr<SubmapManager> submap, std::shared_ptr<BIMManager> bim, FRGresult results) {
	pcl::PointCloud<PointT>::Ptr submapPcd = submap->GetRawPcd();
	pcl::PointCloud<PointT>::Ptr bimPcd = bim->GetPcd3d();
	
	pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
	float leaf_size = 0.25;
	voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxelGrid.setInputCloud(submapPcd);
	voxelGrid.filter(*submapPcd);
	
	voxelGrid.setInputCloud(bimPcd);
	voxelGrid.filter(*bimPcd);
	
	pcl::PointCloud<PointT>::Ptr submapPcdTrans(new pcl::PointCloud<PointT>);
	Eigen::Matrix4d tf = results.tf * submap->GetPcd2d().GetT_rp();
	pcl::transformPointCloud(*submapPcd, *submapPcdTrans, tf);
	
	pcl::PointCloud<PointT>::Ptr cropped_bimPcd(new pcl::PointCloud<PointT>);
	pcl::CropBox<PointT> cropFilter;
	cropFilter.setInputCloud(bimPcd);
//get the bounding box of the submapPcdTrans
	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*submapPcdTrans, min_pt, max_pt);
	Eigen::Vector4f center = (min_pt + max_pt) / 2;
	Eigen::Vector4f size = max_pt - min_pt + Eigen::Vector4f(1.5, 1.5, 1.5, 0);
	cropFilter.setMin(Eigen::Vector4f(center[0] - size[0] / 2, center[1] - size[1] / 2, center[2] - size[2] / 2, 1));
	cropFilter.setMax(Eigen::Vector4f(center[0] + size[0] / 2, center[1] + size[1] / 2, center[2] + size[2] / 2, 1));
	cropFilter.filter(*cropped_bimPcd);
	double chamfer_dist = CloudDist::chamferDistance(submapPcdTrans, bimPcd, 1.0f);
	double overlap = CloudDist::overlap(submapPcdTrans, bimPcd, 2.0f);
	double entropy = CloudDist::entropyDistance(submapPcdTrans, bimPcd, 2.0f);
	return {chamfer_dist, overlap, entropy};
}

void printEvalResult(int trial, int success) {
	LOG(INFO) << "--------------------------------";
	double rate = (double) success / trial * 100;
	LOG(INFO) << "Success rate: " << rate << "%";
	LOG(INFO) << "Success: " << success;
	LOG(INFO) << "Pairs: " << trial;
	LOG(INFO) << "--------------------------------";
}

void printEvalResultVebose(int trial, int success, int success_voting, std::vector<int> fail_index,
						   std::vector<int> fail_voting_index) {
	LOG(INFO) << "--------------------------------";
	double rate_voting = (double) success_voting / trial * 100;
	double rate = (double) success / trial * 100;
	LOG(INFO) << "Success rate: " << rate << "%";
	LOG(INFO) << "Success voting rate (Recall Bound): " << rate_voting << "%";
	LOG(INFO) << "Success: " << success;
	LOG(INFO) << "Pairs: " << trial;
	LOG(INFO) << "Fail voting index: ";
	std::ostringstream fail_voting_index_stream;
	for (auto index: fail_voting_index) {
		fail_voting_index_stream << index << " ";
	}
	LOG(INFO) << fail_voting_index_stream.str();
	std::ostringstream fail_index_stream;
	LOG(INFO) << "Fail index: ";
	for (auto index: fail_index) {
		fail_index_stream << index << " ";
	}
	LOG(INFO) << fail_index_stream.str();
	LOG(INFO) << "--------------------------------";
	
}

void analyzeMethod(std::shared_ptr<SubmapManager> submap_ptr, std::shared_ptr<BIMManager> bim_ptr, FRGresult results,
				   int &success_voting, std::vector<int> &fail_voting_index) {
	Analysis analysis(*submap_ptr, *bim_ptr, results);
	analysis.collectCorrectCandidates();
	if (vizClusters) {
		analysis.vizHoughSpace();
		analysis.vizTopKKeys();
		analysis.vizTopJKeys();
		analysis.vizClusterVotes();
	}
	if (analyzeVerification) analysis.analyzeVerification();
	if (analysis.result_.correct_candidates.size() > 0) {
		success_voting += 1;
	} else {
		std::cout << "Voting: Fail" << std::endl;
		fail_voting_index.push_back(submap_ptr->index_);
	}
}

int main(int argc, char **argv) {
	std::string mode = argv[1];
	std::string config_path =
			bimreg::WORK_SPACE_PATH + "/configs/interval/15m/2F/building_day.yaml";
	
	if (mode == "benchmark") {
		config_path = bimreg::WORK_SPACE_PATH + argv[2];
	}
	InitGLOG(config_path);
	cfg::readParameters(config_path);
	
	BIMManager bim;
	bim.LoadBim();
	std::vector<SubmapManager> submaps;
    std::string Dir = getROOT() + "processed";
    std::string seq = "/" + cfg::seq + "/" + std::to_string(cfg::interval) + "m";
	std::string seqDir = Dir + seq + "/occupied_pcd_flatten";
	std::cout << seqDir << ": " << countFilesInDirectory(seqDir) << std::endl;
	
	if (mode == "single") {
		int index = std::stoi(argv[2]);
		if (argc == 4) {
			if (std::string(argv[3]) == "vizcluster") {
				vizClusters = true;
			} else if (std::string(argv[3]) == "verify") {
				analyzeVerification = true;
			} else if (std::string(argv[3]) == "vizcluster_verify") {
				vizClusters = true;
				analyzeVerification = true;
			}
		}
		SubmapManager submap(index);
		submap.LoadSubmap();
		submaps.push_back(submap);
	} else if (mode == "seq" || mode == "benchmark") {
		int fileCount = countFilesInDirectory(seqDir);
		for (int i: tq::trange(fileCount)) {
			SubmapManager submap(i);
			submap.LoadSubmap();
			submaps.push_back(submap);
		}
	}
	int pair = 0;
	int success = 0;
	int success_voting = 0;
	std::vector<int> fail_index;
	std::vector<int> fail_voting_index;
	LOG(INFO) << "Start Evaluation" << "--------------------------------";
	LOG(INFO) << "Pair" << "-"
			   << "Corners" << "-" << "GeomSets" << "-" << "Correspondences" << "-"<< "HoughSpace" << "-"
			  << "Optimal Score" << "-"
			  << "isSuccess";
	std::string isSuccess;
	std::shared_ptr<BIMManager> bim_ptr = std::make_shared<BIMManager>(bim);
	for (auto submap: submaps) {
		auto results = bimreg::GlobalRegistration(submap, bim);
		
		std::shared_ptr<SubmapManager> submap_ptr = std::make_shared<SubmapManager>(submap);
		analyzeMethod(submap_ptr, bim_ptr, results, success_voting, fail_voting_index);

		if (bimreg::evaluate(results.tf, submap)) {
			success += 1;
			isSuccess = "1";
		} else {
			fail_index.push_back(submap.index_);
			isSuccess = "0";
		}
		pair += 1;
		LOG(INFO) << pair << ", " << submap.GetCorners().size() << ", " << submap.GetGeomSets().size() << ", "
				  << results.geoCorresSize << ", " << results.hough.rawHoughSize << ", " << results.OptimalScore << ", "
				  << isSuccess << ";";
	}
	printEvalResultVebose(pair, success, success_voting, fail_index, fail_voting_index);
//	printEvalResult(pair, success);
	timer.printSummary();
	
	
	return 0;
}