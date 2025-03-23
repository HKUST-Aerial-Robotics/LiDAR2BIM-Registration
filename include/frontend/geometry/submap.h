#ifndef BIMREG_SUBMAP_H
#define BIMREG_SUBMAP_H

#include "frontend/geometry/bim.h"
#include "utils/cfg.h"
#include "global_definition/global_definition.h"
#include "frontend/geometry/point_cloud.h"


class SubmapManager : public BaseMapManager {
public:
    SubmapManager(double index) : BaseMapManager(), index_(index), freePcd_(new pcl::PointCloud<PointT>) {
        std::string rawDir = getROOT();
        std::string Dir = getROOT() + "processed";
        std::string seq = "/" + cfg::seq + "/" + std::to_string(cfg::interval) + "m";
        seqDir = Dir + seq;
		rawSeqDir = rawDir + "/" + cfg::seq + "/" +"submap3d"+ "/" + std::to_string(cfg::interval) + "m";
	}

    void LoadSubmap();

    pcl::PointCloud<PointT>::Ptr GetFreePcd() const { return freePcd_; }
	pcl::PointCloud<PointT>::Ptr GetRawPcd() const { return rawPcd_; }
	PCD2d GetPcd2d() const { return pcd2d_; }

    std::vector<double> LoadGT(std::string gtFile);

    void setT_rp();

    int index_;
    std::vector<double> gt_;
private:
	std::string rawSeqDir;
	std::string seqDir;
    pcl::PointCloud<PointT>::Ptr freePcd_;
	pcl::PointCloud<PointT>::Ptr rawPcd_;
	pcl::PointCloud<PointT>::Ptr rawWallPcd_;
	pcl::PointCloud<PointT>::Ptr rawgroundPcd_;
	PCD2d pcd2d_;
};

#endif //BIMREG_SUBMAP_H
