/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_MATCH_H
#define BIMREG_MATCH_H

#include <iostream>
#include <vector>
#include <variant>
#include "frontend/feature/descriptor.h"

namespace geomset {
    using Corres = std::vector<std::pair<Eigen::Matrix<double, 3, 2>, Eigen::Matrix<double, 3, 2>>>;
    using CorresWithID = std::vector<std::tuple<Eigen::Matrix<double, 3, 2>, Eigen::Matrix<double, 3, 2>, int>>;
    using TriTableManagerPtr = std::shared_ptr<geomset::TriDescHashTable>;
    using SrcTriIndices = std::vector<int>;
    using Results2d = std::vector<std::tuple<double, double, double>>;

    void getCorrespondences(
            CorresWithID &geomSetsCorrespondences,
            const TriTableManagerPtr &srcTableManager,
            const TriTableManagerPtr &tgtTableManager, int width = 1);
	


    void solveCorrespondences(CorresWithID &geomSetsCorrespondences, Results2d &results);
}
#endif //BIMREG_MATCH_H
