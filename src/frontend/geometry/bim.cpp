#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <memory>
#include <utils/cfg.h>
#include "frontend/geometry/bim.h"
#include "frontend/geometry/lineseg.h"

#include <omp.h> //

BaseMapManager::BaseMapManager() : hashTable_(nullptr), occupiedPcd_(nullptr), lines_(nullptr), raster_(nullptr) {}

pcl::PointCloud<PointT>::Ptr
BaseMapManager::LoadPcd(const std::string &path, const Eigen::Vector3d &color) {
	if (std::filesystem::exists(path)) {
		pcl::PointCloud<PointT>::Ptr pcd(new pcl::PointCloud<PointT>());
		if (pcl::io::loadPCDFile<PointT>(path, *pcd) == -1) {
			throw std::runtime_error("Couldn't read file " + path);
		}
		return pcd;
	} else {
		throw std::runtime_error("PCD file not found: " + path);
	}
}

std::shared_ptr<LineSegments> BaseMapManager::LoadLines(const std::string &linesegsFile) {
	if (std::filesystem::exists(linesegsFile)) {
		auto linesegs = std::make_shared<LineSegments>();
		linesegs->readFromFile(linesegsFile);
		return linesegs;
	} else {
		throw std::runtime_error("LineSeg.txt not found, copy bimLineSeg.txt to the " + linesegsFile);
	}
}

void BaseMapManager::ConstructHashTable(bool augmented) {
	std::cout << "Constructing hash table..." << std::endl;
	corners_ = ExtractCorners();
	double radius = cfg::max_side_length;
	hashTable_ = std::make_unique<geomset::TriDescHashTable>(cfg::lengthRes, cfg::angleRes, radius);
	geomSets_ = hashTable_->buildGeomSet(corners_, {});
	hashTable_->buildHashTable(geomSets_, augmented);
	
}

void BaseMapManager::SaveHashTable(const std::string &filePath) const {
	hashTable_->printInfo();
	hashTable_->saveToFile(filePath);
	std::cout << "Hash table saved to " << filePath << std::endl;
}

void BaseMapManager::SaveHashNeighTable(const std::string &filePath) const {
	hashTable_->printInfo();
	hashTable_->savehashNeighTable(filePath);
	std::cout << "Hash Neigh table saved to " << filePath << std::endl;
}

void BaseMapManager::SaveHashIntTable(const std::string &filePath) const {
	hashTable_->savehashIntTable(filePath);
	std::cout << "Hash Int table saved to " << filePath << std::endl;
}

void BaseMapManager::LoadHashTable(const std::string &filePath) {
	std::cout << "Loading hash table from file: " << filePath << std::endl;
	hashTable_ = std::make_unique<geomset::TriDescHashTable>(cfg::lengthRes, cfg::angleRes);
	hashTable_->loadFromFile(filePath);
	hashTable_->printInfo();
}

void BaseMapManager::LoadHashNeighTable(const std::string &filePath) {
	std::cout << "Loading hash neigh table from file: " << filePath << std::endl;
	hashTable_->loadhashNeighTable(filePath);
}

void BaseMapManager::LoadHashIntTable(const std::string &filePath) {
	std::cout << "Loading hash int table from file: " << filePath << std::endl;
	hashTable_->loadhashIntTable(filePath);
}

CornerVector BaseMapManager::ExtractCorners() {
	lines_->extend(2.0);
	auto corners = lines_->intersections(0.7);
	lines_->extend(-2.0);
	return corners;
}

BIMManager::BIMManager()
		: BaseMapManager(),
		  bimDir(getROOT() + "BIM/" + cfg::floor), pcd(new pcl::PointCloud<PointT>()), pcd3d(new pcl::PointCloud<PointT>()) {
}

void BIMManager::LoadBim(bool loadHashTable) {
	// PCD
	std::string pcdFile = bimDir + "/bim2d.pcd";
	LoadPcd(pcdFile);
	std::string pcd3dFile = bimDir + "/bim.pcd";
	LoadPcd3d(pcd3dFile);
	// Lines
	std::string lineSegsFile = bimDir + "/bimLineSeg.txt";
	lines_ = LoadLines(lineSegsFile);
	
	
	// Hash Table
	if (loadHashTable) {
		std::string hashTableFile = bimDir + "/bimHashTable.bin";
		std::string hashNeighTableFile = bimDir + "/bimHashNeighTable.bin";
		std::string hashIntTableFile = bimDir + "/bimHashIntTable.bin";
		bool augmented = cfg::augmented;
		if (std::filesystem::exists(hashTableFile)) {
			LoadHashTable(hashTableFile);
			if (augmented) {
				LoadHashNeighTable(hashNeighTableFile);
				LoadHashIntTable(hashIntTableFile);
			}
		} else {
			ConstructHashTable(augmented);
			SaveHashTable(hashTableFile);
			if (augmented) {
				SaveHashNeighTable(hashNeighTableFile);
				SaveHashIntTable(hashIntTableFile);
			}
		}
	}
	
	// Raster
	LoadRaster();
}

pcl::PointCloud<PointT>::Ptr BIMManager::LoadPcd(const std::string &path, const Eigen::Vector3d &color) {
	if (std::filesystem::exists(path)) {
		std::cout << "Loading BIM PCD file: " << path << std::endl;
		if (pcl::io::loadPCDFile<PointT>(path, *pcd) == -1) {
			throw std::runtime_error("Couldn't read file " + path);
		}
		return pcd;
	} else {
		throw std::runtime_error("BIM PCD file not found: " + path);
	}
}
pcl::PointCloud<PointT>::Ptr BIMManager::LoadPcd3d(const std::string &path, const Eigen::Vector3d &color) {
	if (std::filesystem::exists(path)) {
		std::cout << "Loading BIM PCD3D file: " << path << std::endl;
		if (pcl::io::loadPCDFile<PointT>(path, *pcd3d) == -1) {
			throw std::runtime_error("Couldn't read file " + path);
		}
		return pcd3d;
	} else {
		throw std::runtime_error("BIM PCD file not found: " + path);
	}
}

CornerVector BIMManager::ExtractCorners() {
	lines_->extend(2.0);
	auto corners = lines_->intersections(0.1);
	std::cout << "bim nms_threshold: " << 0.1 << std::endl;
	lines_->extend(-2.0);
	return corners;
}

Eigen::MatrixXd FilterGroundPoints(const Eigen::MatrixXd &points, double minX, double maxX, double minY, double maxY) {
	Eigen::Array<bool, Eigen::Dynamic, 1> groundMask = (points.col(2).array() < 0.01) &&
													   (points.col(0).array() > minX) &&
													   (points.col(0).array() < maxX) &&
													   (points.col(1).array() > minY) && (points.col(1).array() < maxY);
	int count = groundMask.count();
	Eigen::MatrixXd groundPoints(count, points.cols());
	int index = 0;
	for (int i = 0; i < points.rows(); ++i) {
		if (groundMask(i)) {
			groundPoints.row(index++) = points.row(i);
		}
	}
	return groundPoints;
}

std::vector<Eigen::Vector2d> FilterGroundPoints(const std::vector<Eigen::Vector2d> &points, double minX, double maxX,
												double minY, double maxY) {
	std::vector<Eigen::Vector2d> groundPoints;
	for (const auto &point: points) {
		if (point[0] > minX && point[0] < maxX && point[1] > minY && point[1] < maxY) {
			groundPoints.push_back(point);
		}
	}
	return groundPoints;
}

void BIMManager::LoadRaster() {
	// Implementation of loading raster data.
	std::vector<Eigen::Matrix2d> lineset = lines_->get3DEigen();
	double minX, minY, maxX, maxY;
	minX = minY = std::numeric_limits<double>::max();
	maxX = maxY = std::numeric_limits<double>::min();
	for (const auto &line: lineset) {
		for (int i = 0; i < line.rows(); ++i) {
			minX = std::min(minX, line(i, 0));
			minY = std::min(minY, line(i, 1));
			maxX = std::max(maxX, line(i, 0));
			maxY = std::max(maxY, line(i, 1));
		}
	}
	std::vector<Eigen::Vector2d> bimPoints;
	PointCloudToEigen(pcd, bimPoints);
	raster_ = std::make_shared<PointRaster>(bimPoints, cfg::gridSize);
	raster_p_ = std::make_shared<PointRaster>(bimPoints, cfg::gridSize);
	raster_->rasterize();
	raster_p_->rasterize();
	int dilateWidth = static_cast<int>(cfg::sdfDist / cfg::gridSize); //sdfDist: 最外侧BIM点云的栅格中心点算起,unit: m
	double maxVal = cfg::sdfDist / cfg::gridSize + 1;
	raster_->dilate(dilateWidth);
	raster_->cv2DistanceTransform(maxVal, cfg::sigma);
//  for debug visualization
//    	auto gridsNum = raster_->gridsNum_;
//    	for (const auto &pair: gridsNum) {
//    		if (pair.second < 0) {
//    			std::cout << pair.first.transpose() << " " << pair.second << std::endl;
//    		}
//    	}
//    	raster_->visualize();
//    	auto gridsNum = raster_p_->gridsNum_;
//    	for (const auto &pair: gridsNum) {
//    		if (pair.second > 0) {
//    			std::cout << pair.first.transpose() << " " << pair.second << std::endl;
//    		}
//    	}
//		raster_p_->visualize();
}
