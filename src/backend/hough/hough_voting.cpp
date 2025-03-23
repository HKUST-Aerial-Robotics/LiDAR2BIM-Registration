#include "backend/hough/hough_voting.h"
#include "utils/cfg.h"

namespace hough {
	int banker_round(double value) {
		int int_part = static_cast<int>(value);
		double frac_part = value - int_part;
		
		if (frac_part > 0.5 || (frac_part == 0.5 && (int_part % 2 != 0))) {
			return int_part + 1;
		} else {
			return int_part;
		}
	}
	HoughVote::HoughVote(){
		xyRes_ = cfg::xyRes;
		yawRes_ = cfg::yawRes;
		topLScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
		topKScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
		topJScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
	}
	HoughVote::HoughVote(double xyRes, double yawRes) {
		xyRes_ = xyRes;
		yawRes_ = yawRes;
		topLScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
		topKScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
		topJScoreFunc = [this](const Array3D &key) {
			return houghSpace_[key].size();
		};
	}
	
	std::vector<Eigen::Vector3d>
	HoughVote::vote(const geomset::CorresWithID &geomSetsCorrespondences, const geomset::Results2d &results, int width,
					bool mergeNeigh) {
		timer.tic("initHoughSpace");
		initHoughSpace(geomSetsCorrespondences, results);
		timer.toc("initHoughSpace");
		timer.tic("getTopJKeys");
		getTopJKeys(1, true);
		timer.toc("getTopJKeys");
		std::vector<Eigen::Vector3d> tf_candidates;
		timer.tic("keysToCandidates");
		tf_candidates = keysToCandidates(topJKeys);
		timer.toc("keysToCandidates");
		return tf_candidates;
		
	}
	std::vector<Eigen::Vector3d>
	HoughVote::keysToCandidates(const std::vector<VoteKey> &keys){
		std::vector<Eigen::Vector3d> tf_candidates;
		for (const auto &key: keys) {
			if (houghSpace_[key].size() < 1) {
				continue;
			}
			Eigen::Vector3d mean_pose = Eigen::Vector3d::Zero();
			for (const auto &pair: houghSpace_[key]) {
				const auto &pose2d = pair.second[0];
				mean_pose += Eigen::Vector3d(pose2d.x, pose2d.y, pose2d.yaw);
			}
			mean_pose /= houghSpace_[key].size();
			tf_candidates.push_back(mean_pose);
		}
		return tf_candidates;
	}
	
	std::vector<Eigen::Vector3d>
	HoughVote::keysToCandidatesSVD(const std::vector<VoteKey> &keys){
		std::vector<Eigen::Vector3d> tf_candidates;
		for (const auto &key: keys) {
			if (houghSpace_[key].size() < 1) {
				continue;
			}
			Eigen::MatrixXd source;
			Eigen::MatrixXd target;
			for (const auto &pair: houghSpace_[key]) {
				const auto &pose2d = pair.second[0];
				source.conservativeResize(source.rows() + 3, 2);
				source.row(source.rows() - 3) << pose2d.getSrcTriplets().row(0);
				source.row(source.rows() - 2) << pose2d.getSrcTriplets().row(1);
				source.row(source.rows() - 1) << pose2d.getSrcTriplets().row(2);
				target.conservativeResize(target.rows() + 3, 2);
				target.row(target.rows() - 3) << pose2d.getTgtTriplets().row(0);
				target.row(target.rows() - 2) << pose2d.getTgtTriplets().row(1);
				target.row(target.rows() - 1) << pose2d.getTgtTriplets().row(2);
			}
			Eigen::Vector3d svd_result = svd2d(source, target);
			tf_candidates.push_back(svd_result);
		}
		return tf_candidates;
	}
	
	Array3D HoughVote::tfToVoteKey(const std::tuple<double, double, double> &tf) {
		Array3D key;
		key[0] = static_cast<int>(std::round(std::get<0>(tf) / cfg::xyRes));
		key[1] = static_cast<int>(std::round(std::get<1>(tf) / cfg::xyRes));
		key[2] = static_cast<int>(std::round(std::get<2>(tf) / cfg::yawRes));
		return key;
	}
	
	void
	HoughVote::initHoughSpace(const geomset::CorresWithID &geomSetsCorrespondences, const geomset::Results2d &results) {
		//InitHoughSpace = {src_triplet_indice: [pose2d_1, pose2d_2, ...]}
		for (size_t i = 0; i < geomSetsCorrespondences.size(); ++i) {
			const auto &geomSetCorrespondence = geomSetsCorrespondences[i];
			const auto &srcTriIndex = std::get<2>(geomSetCorrespondence);
			const auto &result = results[i];
			
			if (std::isnan(std::get<0>(result))) {
				continue;
			}
			Array3D voteKey = tfToVoteKey(result);
			Eigen::Matrix<double, 3, 2> source = std::get<0>(geomSetCorrespondence);
			Eigen::Matrix<double, 3, 2> target = std::get<1>(geomSetCorrespondence);
			Pose2D pose2d(std::get<0>(result), std::get<1>(result), std::get<2>(result));
			pose2d.setSrcTriplets(source);
//			pose2d.setTriplets(source, target);
			houghSpace_[voteKey][srcTriIndex].push_back(pose2d);
		}
	}
	
	void HoughVote::getTopJKeys(int width, bool mergeNeigh) {
		if (mergeNeigh) {
			timer.tic("truncateKeysTopL");
			truncateKeysTopL();
			timer.toc("truncateKeysTopL");
			timer.tic("mergeNeighVoxel");
			mergeNeighVoxel(width);
			timer.toc("mergeNeighVoxel");
		}
		timer.tic("truncateKeysTopK");
		truncateKeysTopK();
		timer.toc("truncateKeysTopK");
		timer.tic("getNMStopKKeys");
		getNMStopKKeys();
		timer.toc("getNMStopKKeys");
		timer.tic("truncateKeysTopJ");
		truncateKeysTopJ();
		timer.toc("truncateKeysTopJ");
	}
	
	
	void HoughVote::truncateKeysTopL() {
		std::vector<Array3D> keys;
		keys.reserve(houghSpace_.size());
		rawHoughSize = houghSpace_.size();
		for (const auto &[key, _]: houghSpace_) {
			keys.push_back(key);
		}
		std::vector<std::pair<Array3D, int>> keySizePairs;
		keySizePairs.reserve(keys.size());
		for (const auto &key: keys) {
			keySizePairs.emplace_back(key, houghSpace_[key].size());
		}
		int topL = std::min(cfg::topL, static_cast<int>(keySizePairs.size()-1));
		if (topL + 1 == 0)
		{
			return;
		}
		std::partial_sort(
				keySizePairs.begin(),
				keySizePairs.begin() + topL+1,
				keySizePairs.end(),
				[](const std::pair<Array3D, int> &a, const std::pair<Array3D, int> &b) {
					return a.second > b.second;
				}
		);
		
		keys.clear();
		keys.reserve(topL);
		for (int i = 0; i < topL; ++i) {
			keys.push_back(keySizePairs[i].first);
		}
		timer.tic("topL reset space");
		std::unordered_map<VoteKey, std::unordered_map<tripletIndex, PoseList>, Array3DHash> newHoughSpace;
		newHoughSpace.reserve(keys.size());
		for (const auto &key: keys) {
			newHoughSpace.emplace(key, std::move(houghSpace_[key]));
		}
		
		houghSpace_ = std::move(newHoughSpace);
		timer.toc("topL reset space");
	
	}
	
	void HoughVote::mergeNeighVoxel(int width) {
		std::unordered_map<VoteKey, std::unordered_map<tripletIndex, PoseList>, Array3DHash> newHoughSpace;
		newHoughSpace.reserve(houghSpace_.size());
		
		for (const auto &[key, srcTripletMap]: houghSpace_) {
			Array3D newKey = key;
			for (int i = -width; i <= width; ++i) {
				for (int j = -width; j <= width; ++j) {
					for (int k = -width; k <= width; ++k) {
						Array3D neighKey = key;
						neighKey[0] += i;
						neighKey[1] += j;
						neighKey[2] += k;
						
						auto it = houghSpace_.find(neighKey);
						if (it != houghSpace_.end()) {
							for (const auto &[srcTriIndex, poses]: it->second) {
								newHoughSpace[newKey][srcTriIndex].insert(newHoughSpace[newKey][srcTriIndex].end(),
																		  poses.begin(), poses.end());
							}
						}
					}
				}
			}
		}
		
		houghSpace_ = std::move(newHoughSpace);
	}
	
	void HoughVote::truncateKeysTopK() {
		topKKeys.reserve(houghSpace_.size());
		for (const auto &[key, _]: houghSpace_) {
			topKKeys.push_back(key);
		}
		int topK = std::min(cfg::topK, static_cast<int>(topKKeys.size()-1));
		if (topK + 1 == 0)
		{
			return;
		}
//		std::sort(topKKeys.begin(), topKKeys.end(), [this](const Array3D &a, const Array3D &b) {
//			return topKScoreFunc(a) > topKScoreFunc(b);
//		});
		std::partial_sort(topKKeys.begin(), topKKeys.begin() + topK+1, topKKeys.end(),
						  [this](const Array3D &a, const Array3D &b) {
							  return topKScoreFunc(a) > topKScoreFunc(b);
						  });
		if (topKKeys.size() > topK) {
			topKKeys.resize(topK);
		}
	}
	
	void HoughVote::truncateKeysTopJ() {
		topJKeys.reserve(NMStopKKeys.size());
		for (const auto &key: NMStopKKeys) {
			topJKeys.push_back(key);
		}
		
		std::vector<std::pair<Array3D, double>> keyScorePairs;
		keyScorePairs.resize(topJKeys.size());
#pragma omp parallel for
		for (size_t i = 0; i < topJKeys.size(); ++i) {
			keyScorePairs[i] = std::make_pair(topJKeys[i], topJScoreFunc(topJKeys[i]));
		}
		int topJ = std::min(cfg::topJ, static_cast<int>(keyScorePairs.size()-1));
		if (topJ + 1 == 0)
		{
			topJKeys = std::move(topKKeys);
			return;
		}
//        std::sort(keyScorePairs.begin(), keyScorePairs.end(),
//                  [](const std::pair<Array3D, double> &a, const std::pair<Array3D, double> &b) {
//                      return a.second > b.second;
//                  });

		std::partial_sort(keyScorePairs.begin(), keyScorePairs.begin() + topJ+1, keyScorePairs.end(),
						  [](const std::pair<Array3D, double> &a, const std::pair<Array3D, double> &b) {
							  return a.second > b.second;
						  });
		topJKeys.clear();
		topJKeys.reserve(topJ);
		for (int i = 0; i < topJ; ++i) {
			topJKeys.push_back(keyScorePairs[i].first);
		}
	}
	
	void HoughVote::initKDTree() {
		keysCloud.pts.resize(topKKeys.size());
		for (size_t i = 0; i < topKKeys.size(); ++i) {
			keysCloud.pts[i].x = static_cast<double>(topKKeys[i][0]);
			keysCloud.pts[i].y = static_cast<double>(topKKeys[i][1]);
			keysCloud.pts[i].z = static_cast<double>(topKKeys[i][2]);
		}
		treePtr = std::make_shared<kdtree>(3, keysCloud, 10);
	}
	
	void HoughVote::getNMStopKKeys() {
		initKDTree();
		uf = UnionFind(topKKeys.size());
		KDTreeUF();
		for (size_t i = 0; i < topKKeys.size(); ++i) {
			Array3D key = topKKeys[i];
			
			int root = uf.find(i);
			if (clusters.find(root) == clusters.end()) {
				clusters[root] = std::vector<Array3D>();
			}
			clusters[root].push_back(key);
		}
		for (const auto &[_, keys]: clusters) {
			Array3D maxKey = *std::max_element(keys.begin(), keys.end(),
											   [this](const Array3D &a, const Array3D &b) {
												   return houghSpace_.at(a).size() <= houghSpace_.at(b).size();
											   });
			NMStopKKeys.push_back(maxKey);
			
		}
		
	}
	
	void HoughVote::KDTreeUF() {
		double radius = cfg::NeighThreshold + 0.01;
		
		for (size_t i = 0; i < topKKeys.size(); ++i) {
			std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
			double query_pt[3] = {static_cast<double>(topKKeys[i][0]), static_cast<double>(topKKeys[i][1]),
								  static_cast<double>(topKKeys[i][2])};
			treePtr->radiusSearch(&query_pt[0], radius * radius, ret_matches);
			std::vector<uint32_t> indices;
			for (const auto &match: ret_matches) {
				indices.push_back(match.first);
			}
			//sort indices
			std::sort(indices.begin(), indices.end());
			for (const auto &j: indices) {
				if (j > i) {
					uf.unionSets(i, j);
				}
			}
		}
		
	}
	
	double HoughVote::unionArea(const VoteKey &key) {
		std::vector<Polygon> polygons;
		polygons.reserve(houghSpace_[key].size());
		for (const auto &[srcTriIndex, poses]: houghSpace_[key]) {
			Polygon P = poses[0].getSrcBoostTriplet();
			polygons.push_back(std::move(P));
		}
		MultiPolygon result;
		for (const auto &poly: polygons) {
			MultiPolygon temp_result;
			//			if (boost::geometry::is_valid(poly)) {
			boost::geometry::union_(result, poly, temp_result);
			result = std::move(temp_result);
			//			}
		}
		return boost::geometry::area(result);
		
	}
} //namespace hough