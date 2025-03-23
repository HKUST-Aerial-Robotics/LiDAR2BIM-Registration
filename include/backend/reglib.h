/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_REGLIB_H
#define BIMREG_REGLIB_H
#include "utils/cfg.h"
#include "frontend/geometry/bim.h"
#include "frontend/geometry/submap.h"
#include "backend/hough/hough_voting.h"
#include "backend/hough/verification.h"
class FRGresult {
public:
	FRGresult() {
		tf = Eigen::Matrix4d::Identity();
		triangle_inliers = 0;
		valid = true;
	}

public:
	Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
	double OptimalScore = 0;
	int geoCorresSize = 0;
	std::vector<Eigen::Matrix4d> candidates;
	std::vector<Eigen::Matrix4d> correct_candidates;
	hough::HoughVote hough;
	Verification verification;
	int triangle_inliers;
	double feature_time = 0;
	double tf_solver_time = 0;
	double verify_time = 0;
	double total_time = 0;
	bool valid = true;
};
namespace bimreg{
	bool evaluate(const Eigen::Matrix4d &tf, const SubmapManager &submap);
	FRGresult GlobalRegistration(SubmapManager& submap, const BIMManager& bim);
	
}
#endif //BIMREG_REGLIB_H
