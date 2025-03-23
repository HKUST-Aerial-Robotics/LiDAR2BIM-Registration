#include "backend/reglib.h"
#include "frontend/feature/match.h"
#include "backend/hough/hough_voting.h"
#include "backend/hough/verification.h"
#include "utils/utils.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <unordered_set>
#include <Eigen/Dense>

namespace bimreg {
	
	// 自定义哈希函数
	struct Vector4dHash {
		std::size_t operator()(const Eigen::Vector4d& vec) const {
			std::size_t seed = 0;
			for (int i = 0; i < vec.size(); ++i) {
				seed ^= std::hash<double>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
			return seed;
		}
	};

// 自定义相等比较函数
	struct Vector4dEqual {
		bool operator()(const Eigen::Vector4d& lhs, const Eigen::Vector4d& rhs) const {
			return lhs.isApprox(rhs, 1e-6);
		}
	};
	void purgeDuplicateCorres(Eigen::MatrixXd &corres) {
		std::unordered_set<Eigen::Vector4d, Vector4dHash, Vector4dEqual> uniqueRows;
		for (int i = 0; i < corres.rows(); ++i) {
			Eigen::Vector4d row = corres.row(i);
			uniqueRows.insert(row);
		}
		corres.resize(uniqueRows.size(), 4);
		int i = 0;
		for (const auto& row : uniqueRows) {
			corres.row(i++) = row;
		}
	}

    bool
    getDiff(const std::vector<double> &gt, const std::vector<double> &est, double trans_thd = 3, double rot_thd = 5) {
        double trans_err = sqrt(pow(gt[0] - est[0], 2) + pow(gt[1] - est[1], 2));
        double rot_err = abs(gt[2] - est[2]);
        if (trans_err < trans_thd && rot_err < rot_thd) {
            return true;
        } else {
            return false;
        }
    }

    bool evaluate(const Eigen::Matrix4d &tf, const SubmapManager &submap) {
        std::vector<double> est = se32xyyaw(tf, true);
        auto gt = submap.gt_;
        return getDiff(gt, est);
    }

    FRGresult GlobalRegistration(SubmapManager &submap, const BIMManager &bim) {


        timer.tic("GlobalRegistration");
        timer.tic("ConstructHashTable");
        submap.ConstructHashTable();
        const auto &srcTableManager = submap.GetHashTable();
        const auto &tgtTableManager = bim.GetHashTable();
        timer.toc("ConstructHashTable");
        timer.tic("getCorrespondences");
        geomset::CorresWithID geoCorres;
        geomset::getCorrespondences(geoCorres, srcTableManager, tgtTableManager, 1);
        std::cout << "Correspondences: " << geoCorres.size() << std::endl;
        timer.toc("getCorrespondences");

		if (geoCorres.size() == 0) {
			FRGresult results;
			results.OptimalScore = -std::numeric_limits<double>::infinity();
			results.geoCorresSize = 0;
			return results;
		}
		geomset::Results2d solve2dResults;
        solve2dResults.reserve(geoCorres.size());
        timer.tic("solveCorrespondences");
        geomset::solveCorrespondences(geoCorres, solve2dResults);
        timer.toc("solveCorrespondences");

        timer.tic("HoughVote");
        hough::HoughVote houghVote(cfg::xyRes, cfg::yawRes);
        auto tfCandidates = houghVote.vote(geoCorres, solve2dResults, 1, true);
        timer.toc("HoughVote");
        timer.tic("Verification");
        timer.tic("Build Raster");
        Verification verification(submap, bim.GetRaster(), bim.GetRasterPunish());
        timer.toc("Build Raster");
        auto verifyResult = verification.FindTheOptimal(tfCandidates);
        timer.toc("Verification");
        timer.toc("GlobalRegistration");
        FRGresult results;
        results.tf = verifyResult.first;
		results.OptimalScore = verifyResult.second;
		results.geoCorresSize = geoCorres.size();
		for (const auto &tf: tfCandidates) {
			auto candidate = xyyawToSe3(tf[0], tf[1], tf[2], true);
			results.candidates.push_back(candidate);
		}
		results.hough = houghVote;
		results.verification = verification;
        return results;

    }

}