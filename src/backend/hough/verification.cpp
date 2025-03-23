#include "backend/hough/verification.h"
#include <numeric>

#include <pcl/filters/voxel_grid.h>

Verification::Verification(const SubmapManager &submap, std::shared_ptr<PointRaster> bim_raster, std::shared_ptr<PointRaster> bim_raster_punish)
        : bim_raster_(std::move(bim_raster)), bim_raster_p_(std::move(bim_raster_punish)) {
	auto pcd = submap.GetOccupiedPcd();
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(pcd);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);
	sor.filter(*pcd);
	std::vector<Eigen::Vector2d> submap_points;
	PointCloudToEigen(pcd, submap_points);
	submap_points_ = submap_points;

    std::vector<Eigen::Vector2d> groundPoints;
    PointCloudToEigen(submap.GetFreePcd(), groundPoints);

    PointRaster submap_ground_raster(groundPoints);
    submap_ground_raster.rasterize();
    submap_ground_raster.erode(cfg::freeErodeWidth);
    submap_ground_ = submap_ground_raster.getCenters();
}

double Verification::Score(const Eigen::Matrix4d &tf) const {
    std::vector<Eigen::Vector2d> transformed_points;
    for (const auto &point: submap_points_) {
        transformed_points.emplace_back(tf.topLeftCorner<2, 2>() * point + tf.topRightCorner<2, 1>());
    }
    double total_score = 0;
    auto grids = bim_raster_->pointToGrid(transformed_points);
    for (const auto &grid: grids) {
        if (bim_raster_->grids_.find(grid) != bim_raster_->grids_.end()) {
            double score = bim_raster_->getObsNum(grid);
            total_score += score;
        }
    }
	return total_score;
}
double Verification::PunishScore(const Eigen::Matrix4d &tf) const {
    if (submap_ground_.size() == 0) {
        return 0;
    }

    std::vector<Eigen::Vector2d> transformed_ground;
    for (const auto &point: submap_ground_) {
        transformed_ground.emplace_back(tf.topLeftCorner<2, 2>() * point + tf.topRightCorner<2, 1>());
    }

    double total_score = 0;
    auto grids = bim_raster_->pointToGrid(transformed_ground);
    for (const auto &grid: grids) {
        if (bim_raster_->grids_.find(grid) != bim_raster_->grids_.end()) {
            double score = bim_raster_->getObsNum(grid);
                total_score += score;
        }
    }
    return total_score;
}
std::pair<Eigen::Matrix4d, double>
Verification::FindTheOptimal(const std::vector<Eigen::Vector3d> &tf_candidates) const {
    Eigen::Matrix4d optimal_tf;
    double optimal_score = -std::numeric_limits<double>::infinity();
#pragma omp parallel for
    for (const auto &tf_candidate: tf_candidates) {
        Eigen::Matrix4d tf = xyyawToSe3(tf_candidate[0], tf_candidate[1], tf_candidate[2], true);
        double score = Score(tf);
        double punish_score = PunishScore(tf);
		score -= punish_score * cfg::lamda;
		if (score > optimal_score) {
            optimal_tf = tf;
            optimal_score = score;
        }
    }
	optimal_score = optimal_score / submap_points_.size(); //normalize the score
	if (optimal_score == -std::numeric_limits<double>::infinity()) {
		optimal_tf = Eigen::Matrix4d::Identity();
	}
    return {optimal_tf, optimal_score};
}

Verification::Verification() {
	bim_raster_ = nullptr;}
