#include "frontend/feature/descriptor.h"
#include "frontend/geometry/lineseg.h"
#include <Eigen/Dense>
#include <cmath>
#include <unordered_set>
#include <tuple>
#include <stdexcept>
#include <iostream>
#include <functional>
#include <nanoflann.hpp>
#include "utils/tqdm.hpp"
#include "utils/utils.h"

namespace geomset {
	
	template<typename T>
	struct PointCloud {
		struct Point {
			T x, y;
		};
		
		using coord_t = T;  //!< The type of each coordinate
		
		std::vector<Point> pts;
		
		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size(); }
		
		// Returns the dim'th component of the idx'th point in the class:
		inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
			if (dim == 0)
				return pts[idx].x;
			else if (dim == 1)
				return pts[idx].y;
			else
				throw std::out_of_range("dim should be 0 or 1 for 2D points");
		}
		
		// Optional bounding-box computation: return false to default to a standard
		// bbox computation loop.
		// Return true if the BBOX was already computed by the class and returned
		// in "bb" so it can be avoided to redo it again. Look at bb.size() to
		// find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template<class BBOX>
		bool kdtree_get_bbox(BBOX & /* bb */) const {
			return false;
		}
	};
	
	CornerTriplets
	TriDescHashTable::buildTriplets(const CornerVector &corners) {
		if (corners.size() < 3) {
			return {};
		}
		
		PointCloud<double> cloud;
		cloud.pts.resize(corners.size());
		for (size_t i = 0; i < corners.size(); ++i) {
			cloud.pts[i].x = corners[i]->x_;
			cloud.pts[i].y = corners[i]->y_;
		}
		using kdtree_2d = nanoflann::KDTreeSingleIndexAdaptor<
				nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
				PointCloud<double>, 2 /* dim */>;
		kdtree_2d tree(2 /*dim*/, cloud, {10 /* max leaf */});
		
		CornerTriplets triplets;
		for (size_t i = 0; i < corners.size(); ++i) {
			std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
			double query_pt[2] = {corners[i]->x_, corners[i]->y_};
			size_t nMatches = tree.radiusSearch(&query_pt[0], radius_ * radius_, ret_matches);
			std::vector<uint32_t> indices;
			for (const auto &match: ret_matches) {
				indices.push_back(match.first);
			}
			//sort indices
			std::sort(indices.begin(), indices.end());
			if (nMatches >= 3) {
				for (auto j: indices) {
					for (auto k: indices) {
						if (i < j && j < k) {
							triplets.push_back({corners[i], corners[j], corners[k]});
						}
					}
				}
			}
		}
		
		return triplets;
	}
	
	// Build geometric set from corners
	CornerTriplets
	TriDescHashTable::buildGeomSet(const CornerVector &corners,
								   const std::vector<std::shared_ptr<LineSegment>> &lineSegments) {
		return buildTriplets(corners);
	}
	
	bool TriDescHashTable::isValidTriangle(const std::vector<double> &sides) const {
		double a = sides[0], b = sides[1], c = sides[2];
		if (std::abs(b * c) < 1e-6 || c > 37.5) {
			return false;
		}
		double cosValue = (b * b + c * c - a * a) / (2 * b * c);
		return std::abs(cosValue) <= cosThreshold;
	}
	
	Eigen::Array<double, 6, 1> TriDescHashTable::descriptorExtraction(const std::vector<double> &sides,
																	  const CornerPtr &A,
																	  const CornerPtr &B,
																	  const CornerPtr &C) {
		if (!isValidTriangle(sides)) {
			return Eigen::Array<double, 6, 1>::Zero();
		}
		
		std::vector<double> angles;
		LineSegment AB;
		LineSegment BC;
		LineSegment CA;
		
		AB.fromCorners(A, B);
		BC.fromCorners(B, C);
		CA.fromCorners(C, A);
		
		for (const auto &[line, corner]: {std::make_pair(AB, A), std::make_pair(BC, B), std::make_pair(CA, C)}) {
			double angle = std::numeric_limits<double>::max();
			for (const auto &wallLine: *corner->getLines()) {
				angle = std::min(angle, wallLine.angle(line));
			}
			angles.push_back(angle);
		}
		
		Eigen::Array<double, 6, 1> feature;
		feature << sides[0], sides[1], sides[2], angles[0], angles[1], angles[2];
		return feature;
	}
	
	
	// Build hash table from geometric sets
	void
	TriDescHashTable::buildHashTable(const CornerTriplets &geomSet, bool augmented /*= false*/) {
		hashTable_.clear();
		for (const auto &cornerTriplet: tq::tqdm(geomSet)) {
			Eigen::Matrix<double, 3, 2> triplet;
			for (int i = 0; i < 3; ++i) {
				triplet(i, 0) = cornerTriplet[i]->x_;
				triplet(i, 1) = cornerTriplet[i]->y_;
			}
			
			std::vector<double> sides(3);
			for (int i = 0; i < 3; ++i) {
				sides[i] = (triplet.row(i) - triplet.row((i + 1) % 3)).norm();
			}
			std::array<CornerPtr, 3> corners{cornerTriplet[0], cornerTriplet[1], cornerTriplet[2]};
			std::array<CornerPtr, 3> sortedCorners;
			std::array<int, 3> indices = {0, 1, 2};
			std::sort(indices.begin(), indices.end(), [&sides](int i, int j) {
				return sides[i] < sides[j];
			});
			for (int i = 0; i < 3; ++i) {
				sortedCorners[i] = corners[indices[i]];
			}
			
			std::sort(sides.begin(), sides.end());
			auto feature = descriptorExtraction(sides, sortedCorners[0], sortedCorners[1], sortedCorners[2]);
			Array6D hashKey = hash(feature);
			if (hashKey == Array6D()) {
				continue;
			}
			
			// If the key is not in the hash table, create a new entry automatically. This is the feature of std::unordered_map
			auto &entries = hashTable_[hashKey];
			Eigen::Matrix<double, 3, 2> tripletSorted;
			tripletSorted << sortedCorners[0]->x_, sortedCorners[0]->y_,
					sortedCorners[1]->x_, sortedCorners[1]->y_,
					sortedCorners[2]->x_, sortedCorners[2]->y_;
			entries.push_back(tripletSorted);
		}
		if (augmented) {
			// create DescNeighHash hashNeighTable_. for each key, we add the neighbouring keys to the hash table
			auto keyInRange = [&](const Array6D &key, int width, bool reverse = false) {
				int s1 = key[0], s2 = key[1], s3 = key[2];
				int a1 = key[3], a2 = key[4], a3 = key[5];
				
				std::vector<Array6D> newKeys;
				for (int i = -width; i <= width; ++i) {
					for (int j = -width; j <= width; ++j) {
						for (int k = -width; k <= width; ++k) {
							for (int l = -width; l <= width; ++l) {
								for (int m = -width; m <= width; ++m) {
									for (int n = -width; n <= width; ++n) {
										Array6D newKey = {s1 + i, s2 + j, s3 + k, a1 + l, a2 + m, a3 + n};
										//check all the values are positive
										bool valid = true;
										for (int ii = 0; ii < 6; ++ii) {
											if (newKey[ii] < 0) {
												valid = false;
												break;
											}
										}
										if (!reverse) {
											if (valid && hashTable_.find(newKey) != hashTable_.end()) {
												newKeys.push_back(newKey);
											}
										} else {
											if (valid && hashTable_.find(newKey) == hashTable_.end()) {
												newKeys.push_back(newKey);
											}
										}
									}
								}
							}
						}
					}
				}
				
				if (!newKeys.empty()) {
					return std::make_pair(true, newKeys);
				} else {
					return std::make_pair(false, std::vector<Array6D>{});
				}
			};
			std::cout << "Augmenting hash table..." << std::endl;
			std::cout << "Original hash table size: " << hashTable_.size() << std::endl;
			int id = 1;
			std::unordered_map<Array6D, int, Array6DHash> hashTableId;
			for (const auto &key: tq::tqdm(getKeys())) {
				hashTableId[key] = id;
				id++;
			}
			for (const auto &key: tq::tqdm(getKeys())) {
				hashIntTable_[hashTableId[key]] = hashTable_[key];
			}
			for (const auto &key: tq::tqdm(getKeys())) {
				auto [inRange, neighKeys] = keyInRange(key, 1);
				if (inRange) {
					for (const auto &newKey: neighKeys) {
						hashNeighTable_[key].push_back(hashTableId[newKey]);
					}
				}
			}
			std::cout << "Original hashNeighTable size: " << hashNeighTable_.size() << std::endl;
			std::cout << "id: " << id << std::endl;
			std::set<Array6D> newNeighKeys;
			for (const auto &key: tq::tqdm(getKeys())) {
				auto [inRange, neighKeys] = keyInRange(key, 1, true);
				if (inRange) {
					for (const auto &newKey: neighKeys) {
						newNeighKeys.insert(newKey);
					}
				}
			}
			std::cout << "New keys to add: " << newNeighKeys.size() << std::endl;
			std::cout << "id: " << id << std::endl;
			for (const auto &newKey: tq::tqdm(newNeighKeys)) {
				auto [inRange, neighKeys] = keyInRange(newKey, 1);
				if (inRange) {
					for (const auto &key: neighKeys) {
						hashNeighTable_[newKey].push_back(hashTableId[key]);
					}
				}
			}
			std::cout << "New hashNeighTable size: " << hashNeighTable_.size() << std::endl;
			
		}
	}
	
	std::vector<Array6D> TriDescHashTable::getKeys() const {
		std::vector<Array6D> keys;
		for (const auto &pair: hashTable_) {
			keys.push_back(pair.first);
		}
		return keys;
	}
	
	int banker_round(double value) {
		int int_part = static_cast<int>(value);
		double frac_part = value - int_part;
		
		if (frac_part > 0.5 || (frac_part == 0.5 && (int_part % 2 != 0))) {
			return int_part + 1;
		} else {
			return int_part;
		}
	}
	
	// Hash function for feature vector
	Array6D TriDescHashTable::hash(const Eigen::Array<double, 6, 1> &feature) {
		if (feature.size() != 6) {
			throw std::invalid_argument("Feature vector must have 6 elements.");
		}
		
		Array6D hash_key;
		Eigen::Array<double, 6, 1> resArray;
		resArray << lengthRes_, lengthRes_, lengthRes_, angleRes_, angleRes_, angleRes_;
		//		Eigen::Array<int, 6, 1> roundedArray = (feature / resArray).round().cast<int>();
		Eigen::Array<int, 6, 1> roundedArray;
		for (int i = 0; i < 6; ++i) {
			roundedArray[i] = banker_round(feature[i] / resArray[i]);
		}
		hash_key.data = roundedArray;
		
		return hash_key;
	}
	
	std::vector<std::tuple<Array6D, Array6D, int>>
	TriDescHashTable::getCommonKeys(const BaseDescHashTable &tgtDescTable, int width) {
		
		const auto &tgtTable = tgtDescTable.getHashTable();
		if (width < 1) {
			std::unordered_set<Array6D, Array6DHash> keys;
			// here we think the size of source hash table is smaller than the target hash table
			assert(hashTable_.size() < tgtTable.size());
			for (const auto &key: hashTable_) {
				if (tgtTable.find(key.first) != tgtTable.end()) {
					keys.insert(key.first);
				}
			}
			int srcKeyIndex = 1;
			std::vector<std::tuple<Array6D, Array6D, int>> result;
			for (const auto &key: keys) {
				result.emplace_back(key, key, srcKeyIndex);
				srcKeyIndex++;
			}
			return result;
		}
		
		auto keyInRange = [&](const Array6D &key, int width) {
			int s1 = key[0], s2 = key[1], s3 = key[2];
			int a1 = key[3], a2 = key[4], a3 = key[5];
			
			std::vector<Array6D> newKeys;
			for (int i = -width; i <= width; ++i) {
				for (int j = -width; j <= width; ++j) {
					for (int k = -width; k <= width; ++k) {
						for (int l = -width; l <= width; ++l) {
							for (int m = -width; m <= width; ++m) {
								for (int n = -width; n <= width; ++n) {
									Array6D newKey = {s1 + i, s2 + j, s3 + k, a1 + l, a2 + m, a3 + n};
									//check all the values are positive
									bool valid = true;
									for (int ii = 0; ii < 6; ++ii) {
										if (newKey[ii] < 0) {
											valid = false;
											break;
										}
									}
									if (valid && tgtTable.find(newKey) != tgtTable.end()) {
										newKeys.push_back(newKey);
									}
								}
							}
						}
					}
				}
			}
			
			if (!newKeys.empty()) {
				return std::make_pair(true, newKeys);
			} else {
				return std::make_pair(false, std::vector<Array6D>{});
			}
		};
		
		std::vector<std::tuple<Array6D, Array6D, int>> commonKeys;
		int srcKeyIndex = 1;
		for (const auto &key: hashTable_) {
			auto [inRange, tgtKeyList] = keyInRange(key.first, width);
			if (inRange) {
				for (const auto &tgtKey: tgtKeyList) {
					
					commonKeys.emplace_back(key.first, tgtKey, srcKeyIndex);
				}
				srcKeyIndex++;
			}
		}
		return commonKeys;
	}
	
	std::vector<std::tuple<Array6D, int, int>>
	TriDescHashTable::getAugCommonKeys(const BaseDescHashTable &tgtDescTable) {
		
		const auto &tgtTable = tgtDescTable.getHashNeighTable();
		std::vector<std::tuple<Array6D, int, int>> result;
		// here we think the size of source hash table is smaller than the target hash table
		assert(hashTable_.size() < tgtTable.size());
		int srcKeyIndex = 1;
		for (const auto &key: hashTable_) {
			if (tgtTable.find(key.first) != tgtTable.end()) {
				for (const auto &tgtKey: tgtTable.at(key.first)) {
					result.emplace_back(key.first, tgtKey, srcKeyIndex);
				}
				srcKeyIndex++;
			}
		}

		return result;
		
	}
	
	CornerTriplets
	TriDescHashTable2::buildGeomSet(const CornerVector &corners,
									const std::vector<std::shared_ptr<LineSegment>> &lineSegments) {
		return triDescHashTable.buildGeomSet(corners, lineSegments);
	}
	
	std::string TriDescHashTable2::hash(const Eigen::Array<double, 6, 1> &feature) {
		if (feature.size() != 6) {
			throw std::invalid_argument("Feature vector must have 6 elements.");
		}
		if (feature[0] == 0 || feature[1] == 0 || feature[2] == 0) {
			return "";
		}
		Eigen::Array<double, 6, 1> resArray;
		resArray << lengthRes_, lengthRes_, lengthRes_, angleRes_, angleRes_, angleRes_;
		Eigen::Array<int, 6, 1> roundedArray = (feature / resArray).round().cast<int>();
		
		std::string hash_key = std::to_string(roundedArray[0]);
		for (int i = 1; i < 6; ++i) {
			hash_key += "_" + std::to_string(roundedArray[i]);
		}
		return hash_key;
	}
	
	void TriDescHashTable2::buildHashTable(const CornerTriplets &geomSet, bool augmented /*= false*/) {
		hashTable_.clear();
		for (const auto &cornerTriplet: tq::tqdm(geomSet)) {
			Eigen::Matrix<double, 3, 2> triplet;
			for (int i = 0; i < 3; ++i) {
				triplet(i, 0) = cornerTriplet[i]->x_;
				triplet(i, 1) = cornerTriplet[i]->y_;
			}
			
			std::vector<double> sides(3);
			for (int i = 0; i < 3; ++i) {
				sides[i] = (triplet.row(i) - triplet.row((i + 1) % 3)).norm();
			}
			std::array<CornerPtr, 3> corners{cornerTriplet[0], cornerTriplet[1], cornerTriplet[2]};
			std::array<CornerPtr, 3> sortedCorners;
			std::array<int, 3> indices = {0, 1, 2};
			std::sort(indices.begin(), indices.end(), [&sides](int i, int j) {
				return sides[i] < sides[j];
			});
			for (int i = 0; i < 3; ++i) {
				sortedCorners[i] = corners[indices[i]];
			}
			
			std::sort(sides.begin(), sides.end());
			auto feature = triDescHashTable.descriptorExtraction(sides, sortedCorners[0], sortedCorners[1],
																 sortedCorners[2]);
			std::string hashKey = hash(feature);
			if (hashKey == "") {
				continue;
			}
			
			if (stringHashTable_.find(hashKey) == stringHashTable_.end()) {
				stringHashTable_[hashKey] = {};
			}
			Eigen::Matrix<double, 3, 2> tripletSorted;
			tripletSorted << sortedCorners[0]->x_, sortedCorners[0]->y_,
					sortedCorners[1]->x_, sortedCorners[1]->y_,
					sortedCorners[2]->x_, sortedCorners[2]->y_;
			stringHashTable_[hashKey].push_back(tripletSorted);
		}
	}
	
	std::vector<std::tuple<std::string, std::string, int>>
	TriDescHashTable2::getCommonKeys(const BaseDescHashTable &tgtDescTable, int width) {
		std::vector<std::tuple<std::string, std::string, int>> commonKeys;
		return commonKeys;
	}
	
	std::vector<std::pair<std::string, std::string>>
	TriDescHashTable2::getStringCommonKeys(const TriDescHashTable2 &tgtDescTable, int width) const {
		if (width < 1) {
			std::unordered_set<std::string> keys;
			for (const auto &key: stringHashTable_) {
				if (tgtDescTable.getStringHashTable().find(key.first) != tgtDescTable.getStringHashTable().end()) {
					keys.insert(key.first);
				}
			}
			std::vector<std::pair<std::string, std::string>> result;
			for (const auto &key: keys) {
				result.push_back({key, key});
			}
			return result;
		}
		auto tgtTable = tgtDescTable.getStringHashTable();
		auto keyInRange = [&](const std::string &key, int width) {
			std::vector<std::string> newKeys;
			std::vector<std::string> keyParts;
			std::string keyPart;
			for (const auto &c: key) {
				if (c == '_') {
					keyParts.push_back(keyPart);
					keyPart = "";
				} else {
					keyPart += c;
				}
			}
			keyParts.push_back(keyPart);
			std::vector<int> s(3), a(3);
			for (int i = 0; i < 3; ++i) {
				s[i] = std::stoi(keyParts[i]);
				a[i] = std::stoi(keyParts[i + 3]);
			}
			for (int i = -width; i <= width; ++i) {
				for (int j = -width; j <= width; ++j) {
					for (int k = -width; k <= width; ++k) {
						for (int l = -width; l <= width; ++l) {
							for (int m = -width; m <= width; ++m) {
								for (int n = -width; n <= width; ++n) {
									std::string newKey =
											std::to_string(s[0] + i) + "_" + std::to_string(s[1] + j) + "_" +
											std::to_string(s[2] + k) + "_" + std::to_string(a[0] + l) + "_" +
											std::to_string(a[1] + m) + "_" + std::to_string(a[2] + n);
									if (tgtTable.find(newKey) != tgtTable.end()) {
										newKeys.push_back(newKey);
									}
								}
							}
						}
					}
				}
			}
			
			if (!newKeys.empty()) {
				return std::make_pair(true, newKeys);
			} else {
				return std::make_pair(false, std::vector<std::string>{});
			}
		};
		
		std::vector<std::pair<std::string, std::string>> commonKeys;
		for (const auto &key: stringHashTable_) {
			auto [inRange, tgtKeyList] = keyInRange(key.first, width);
			if (inRange) {
				for (const auto &tgtKey: tgtKeyList) {
					commonKeys.push_back({key.first, tgtKey});
				}
			}
		}
		return commonKeys;
	}
	
	void TriDescHashTable2::printInfo() const {
		std::cout << "--------------------------------" << std::endl;
		std::cout << "HashTable Info:" << std::endl;
		std::cout << "Length Resolution: " << lengthRes_ << std::endl;
		std::cout << "Angle Resolution: " << angleRes_ << std::endl;
		std::cout << "Number of Keys: " << stringHashTable_.size() << std::endl;
		
		size_t totalNum = 0;
		for (const auto &pair: stringHashTable_) {
			totalNum += pair.second.size();
		}
		std::cout << "Total Number of GeomSets: " << totalNum << std::endl;
		std::cout << "--------------------------------" << std::endl;
	}
} // namespace geomset