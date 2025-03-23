#ifndef BIMREG_ANALYSIS_H
#define BIMREG_ANALYSIS_H

#include "frontend/geometry/submap.h"
#include "backend/reglib.h"
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
class Analysis {
public:
	Analysis(const SubmapManager &submap, const BIMManager &bim, const FRGresult &result) : submap_(submap), bim_(bim), result_(result) {}
	SubmapManager submap_;
	BIMManager bim_;
	FRGresult result_;
	void collectCorrectCandidates() {
		for (const auto &tf: result_.candidates) {
			if (bimreg::evaluate(tf, submap_)) {
				result_.correct_candidates.push_back(tf);
			}
		}
	}
	void analyzeVerification(){
		std::cout << "gt verification score: " << VerificationEval(xyyawToSe3(submap_.gt_[0], submap_.gt_[1], submap_.gt_[2],
																			  true)) << std::endl;
		for (const auto &tf: result_.correct_candidates) {
			std::cout << "correct candidate verification score: " << VerificationEval(tf) << std::endl;
			VisualizeCandidate(tf);
		}
		std::cout << "Estimate Verification Score: " << VerificationEval(result_.tf) << std::endl;
		VisualizeCandidate(result_.tf);
	}
	double VerificationEval(const Eigen::Matrix4d &tf) const
	{
		double score = result_.verification.Score(tf);
		std::cout << "Verification score: " << score << std::endl;
		double punish_score = result_.verification.PunishScore(tf);
		std::cout << "Verification punish score: " << punish_score << std::endl;
		return score - punish_score;
	}
	
	void VisualizeCandidate(const Eigen::Matrix4d &tf) const
	{
      // use PCL to visualize the candidate
	  auto submapOccupiedPcd = submap_.GetOccupiedPcd();
	  auto submapFreePcd = submap_.GetFreePcd();
	  auto bimPcd = bim_.GetPcd();
	  pcl::PointCloud<PointT>::Ptr submapOccupiedPcdTransformed(new pcl::PointCloud<PointT>);
	  pcl::PointCloud<PointT>::Ptr submapFreePcdTransformed(new pcl::PointCloud<PointT>);
		Eigen::Matrix4d move_z = Eigen::Matrix4d::Identity();
		move_z(2, 3) = 0.3;
		Eigen::Matrix4d adjust_tf = move_z * tf;
	  // transform the submap occupied points
	  pcl::transformPointCloud(*submapOccupiedPcd, *submapOccupiedPcdTransformed, adjust_tf);
	  pcl::transformPointCloud(*submapFreePcd, *submapFreePcdTransformed, adjust_tf);
	  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	  viewer->setBackgroundColor(0, 0, 0);
	  
	  viewer->addPointCloud<PointT>(submapOccupiedPcdTransformed, pcl::visualization::PointCloudColorHandlerCustom<PointT>(submapOccupiedPcdTransformed, 0, 0.651 * 255, 0.929 * 255), "submap");
	  viewer->addPointCloud<PointT>(submapFreePcdTransformed, pcl::visualization::PointCloudColorHandlerCustom<PointT>(submapFreePcdTransformed, 0.929 * 255, 0.651 * 255, 0), "submap_free");
	  viewer->addPointCloud<PointT>(bimPcd, pcl::visualization::PointCloudColorHandlerCustom<PointT>(bimPcd, 255, 255, 255), "bim");
		
	  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "submap");
	  viewer->spin();
	  
	}
	
	void vizHoughSpace(){
		auto houghSpace = result_.hough.getHoughSpace();
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Hough Space"));
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcd->points.resize(houghSpace.size());
		int max_vote = 0;
		std::vector<int> votes;
		int n = 0;
		for (const auto &[key, value]: houghSpace) {
			pcd->points[n].x = static_cast<float>(key[0]);
			pcd->points[n].y = static_cast<float>(key[1]);
			pcd->points[n].z = static_cast<float>(key[2]);
			max_vote = std::max(max_vote, static_cast<int>(value.size()));
			votes.push_back(static_cast<int>(value.size()));
			n += 1;
		}
		// paint the pcd with color according to the number of votes to the maximum number of votes
		for (size_t i = 0; i < pcd->points.size(); ++i) {
			int color = static_cast<int>(255 * votes[i] / max_vote);
			pcd->points[i].r = 50 + color * (255-50) / 255;
			pcd->points[i].g = 0;
			pcd->points[i].b = 243 - color * 243 / 255;
			pcd->points[i].a = color + (255 - color) * 0.3;
		}
		viewer->addPointCloud(pcd, "hough_space");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "hough_space");
		viewer->setBackgroundColor(255, 255, 255);
		viewer->addCoordinateSystem(20.0);
		viewer->spin();
		//save the hough space to a pcd file
		pcl::io::savePCDFileBinary("/home/qzj/code/test/bim-relocalization/data/analysis/topL.pcd", *pcd);
	};
	void vizTopKKeys() {
		auto topKKeys = result_.hough.getTopKKeys();
		auto houghSpace = result_.hough.getHoughSpace();
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Top K Keys"));
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZRGBA>);
		int max_vote = 0;
		std::vector<int> votes;
		pcd->points.resize(topKKeys.size());
		for (size_t i = 0; i < topKKeys.size(); ++i) {
			pcd->points[i].x = static_cast<float>(topKKeys[i][0]);
			pcd->points[i].y = static_cast<float>(topKKeys[i][1]);
			pcd->points[i].z = static_cast<float>(topKKeys[i][2]);
			max_vote = std::max(max_vote, static_cast<int>(houghSpace[topKKeys[i]].size()));
			votes.push_back(static_cast<int>(houghSpace[topKKeys[i]].size()));
		}
		std::cout << "max_vote: " << max_vote << std::endl;
		for (size_t i = 0; i < pcd->points.size(); ++i) {
			int color = static_cast<int>(255 * votes[i] / max_vote);
			pcd->points[i].r = 50 + color * (255-50) / 255;
			pcd->points[i].g = 0;
			pcd->points[i].b = 243 - color * 243 / 255;
			pcd->points[i].a = color + (255 - color) * 0.3;
		}
		viewer->addPointCloud(pcd, "top_k_keys");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "top_k_keys");
		viewer->setBackgroundColor(255, 255, 255);
		viewer->addCoordinateSystem(20.0);
		viewer->spin();
		pcl::io::savePCDFileBinary("/home/qzj/code/test/bim-relocalization/data/analysis/topK.pcd", *pcd);
	}
	
	void vizTopJKeys() {
		auto topJKeys = result_.hough.getTopJKeys();
		auto houghSpace = result_.hough.getHoughSpace();
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Top J Keys"));
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZRGBA>);
		int max_vote = 0;
		std::vector<int> votes;
		pcd->points.resize(topJKeys.size());
		for (size_t i = 0; i < topJKeys.size(); ++i) {
			pcd->points[i].x = static_cast<float>(topJKeys[i][0]);
			pcd->points[i].y = static_cast<float>(topJKeys[i][1]);
			pcd->points[i].z = static_cast<float>(topJKeys[i][2]);
			max_vote = std::max(max_vote, static_cast<int>(houghSpace[topJKeys[i]].size()));
			votes.push_back(static_cast<int>(houghSpace[topJKeys[i]].size()));
		}
		for (size_t i = 0; i < pcd->points.size(); ++i) {
			int color = static_cast<int>(255 * votes[i] / max_vote);
			pcd->points[i].r = 50 + color * (255-50) / 255;
			pcd->points[i].g = 0;
			pcd->points[i].b = 243 - color * 243 / 255;
			pcd->points[i].a = color + (255 - color) * 0.3;
		}
		viewer->addPointCloud(pcd, "top_j_keys");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "top_j_keys");
		viewer->setBackgroundColor(255, 255, 255);
		viewer->addCoordinateSystem(20.0);
		viewer->spin();
		pcl::io::savePCDFileBinary("/home/qzj/code/test/bim-relocalization/data/analysis/topJ.pcd", *pcd);
	}

	void vizClusterVotes() {
		auto clusters = result_.hough.getClusters();
		std::cout << "Number of clusters: " << clusters.size() << std::endl;
		auto NMStopKKeys = result_.hough.fetchNMStopKKeys();
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster Visualization"));

		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pcd(new pcl::PointCloud<pcl::PointXYZ>);
		int cluster_id = 0;
		for (const auto &[_, keys]: clusters) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
			pcd->points.resize(keys.size());
			for (size_t i = 0; i < keys.size(); ++i) {
				pcd->points[i].x = static_cast<float>(keys[i][0]);
				pcd->points[i].y = static_cast<float>(keys[i][1]);
				pcd->points[i].z = static_cast<float>(keys[i][2]);
			}
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(pcd, rand() % 256, rand() % 256, rand() % 256);
			viewer->addPointCloud(pcd, color_handler, "cluster" + std::to_string(cluster_id));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster" + std::to_string(cluster_id));
			++cluster_id;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr nmsk_pcd(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto &key: NMStopKKeys) {
			nmsk_pcd->points.push_back(pcl::PointXYZ(key[0], key[1], key[2]));

		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_nmsk(nmsk_pcd, 10, 0, 0);
		viewer->addPointCloud(nmsk_pcd, color_handler_nmsk, "NMStopKKeys");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "NMStopKKeys");
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr gt_pcd(new pcl::PointCloud<pcl::PointXYZ>);
		auto gt = result_.hough.tfToVoteKey(std::make_tuple(submap_.gt_[0], submap_.gt_[1], submap_.gt_[2]));
		gt_pcd->points.push_back(pcl::PointXYZ(gt[0], gt[1], gt[2]));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_gt(gt_pcd, 255, 0, 0);
		viewer->addPointCloud(gt_pcd, color_handler_gt, "gt");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "gt");
		viewer->setBackgroundColor(255, 255, 255);
		viewer->addCoordinateSystem(20.0);
		viewer->spin();

	}
};
#endif //BIMREG_ANALYSIS_H
