#ifndef BIMREG_BIM_H
#define BIMREG_BIM_H

#include <iostream>
#include <memory>
#include <stdexcept>
#include <filesystem>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <map>
#include <vector>
#include "frontend/geometry/lineseg.h"
#include "frontend/feature/descriptor.h"
#include "utils/cfg.h"
#include "utils/utils.h"
#include "utils/raster_utils.h"
#include "global_definition/global_definition.h"

class BaseMapManager {
public:
    explicit BaseMapManager();

    virtual pcl::PointCloud<PointT>::Ptr
    LoadPcd(const std::string &path, const Eigen::Vector3d &color = Eigen::Vector3d(0.929, 0, 0.651));

    std::shared_ptr<LineSegments> LoadLines(const std::string &linesegsFile);

    void ConstructHashTable(bool augmented = false);

    void SaveHashTable(const std::string &filePath) const;
	void SaveHashNeighTable(const std::string &filePath) const;
	void SaveHashIntTable(const std::string &filePath) const;
	
	
	void LoadHashTable(const std::string &filePath);
	void LoadHashNeighTable(const std::string &filePath);
	void LoadHashIntTable(const std::string &filePath);

    virtual CornerVector ExtractCorners();
	
	pcl::PointCloud<PointT>::Ptr GetOccupiedPcd() const { return occupiedPcd_; }

    std::shared_ptr<LineSegments> GetLines() const { return lines_; }
	
	CornerVector GetCorners() const { return corners_; }
	
	CornerTriplets GetGeomSets() const { return geomSets_; }

    std::shared_ptr<geomset::TriDescHashTable> GetHashTable() const { return hashTable_; }

    std::shared_ptr<geomset::TriDescHashTable2> GetHashTable2() const { return hashTable2_; }
	
	std::shared_ptr<PointRaster> GetRaster() const { return raster_; }
	std::shared_ptr<PointRaster> GetRasterPunish() const { return raster_p_; }
	

protected:
	CornerVector corners_;
	CornerTriplets geomSets_;
    std::shared_ptr<geomset::TriDescHashTable> hashTable_;
    std::shared_ptr<geomset::TriDescHashTable2> hashTable2_;//string key
    pcl::PointCloud<PointT>::Ptr occupiedPcd_;
    std::shared_ptr<LineSegments> lines_;
	std::shared_ptr<PointRaster> raster_;
	std::shared_ptr<PointRaster> raster_p_;
};

class BIMManager : public BaseMapManager {
public:
    BIMManager();

    void LoadBim(bool loadHashTable = true);

    pcl::PointCloud<PointT>::Ptr
    LoadPcd(const std::string &path, const Eigen::Vector3d &color = Eigen::Vector3d(0, 0.651, 0.929));
	pcl::PointCloud<PointT>::Ptr LoadPcd3d(const std::string &path, const Eigen::Vector3d &color = Eigen::Vector3d(0, 0.651, 0.929));
	pcl::PointCloud<PointT>::Ptr GetPcd3d() const { return pcd3d; }
	
	pcl::PointCloud<PointT>::Ptr GetPcd() const { return pcd; }
    CornerVector ExtractCorners();

private:
    std::string bimDir;
    pcl::PointCloud<PointT>::Ptr pcd;
	pcl::PointCloud<PointT>::Ptr pcd3d;
	
	
	//	Raster raster;
    void LoadRaster();
};

#endif //BIMREG_BIM_H
