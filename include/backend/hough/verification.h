#ifndef BIMREG_VERIFICATION_H
#define BIMREG_VERIFICATION_H

#include <vector>
#include <Eigen/Dense>
#include "frontend/geometry/submap.h"
#include "utils/utils.h"
#include "utils/cfg.h"
#include "utils/raster_utils.h"

class Verification {
public:
	Verification(const SubmapManager &submap, std::shared_ptr<PointRaster> bim_raster, std::shared_ptr<PointRaster> bim_raster_punish);
	
	
	Verification();
	double Score(const Eigen::Matrix4d &tf) const;
    double PunishScore(const Eigen::Matrix4d &tf) const;

    std::pair<Eigen::Matrix4d, double> FindTheOptimal(const std::vector<Eigen::Vector3d> &tf_candidates) const;

private:
	std::shared_ptr<PointRaster> bim_raster_;
	std::shared_ptr<PointRaster> bim_raster_p_;

    std::vector<Eigen::Vector2d> submap_points_;
    std::vector<Eigen::Vector2d> submap_ground_;
};

#endif //BIMREG_VERIFICATION_H
