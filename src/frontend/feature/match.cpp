#include "frontend/feature/match.h"
#include "utils/utils.h"
#include "omp.h"

namespace geomset {
    void getCorrespondences(
            CorresWithID &geomSetsCorrespondences,
            const TriTableManagerPtr &srcTableManager,
            const TriTableManagerPtr &tgtTableManager, int width) {
        timer.tic("getCommonKeys");
        const auto &commonKeys = srcTableManager->getCommonKeys(*tgtTableManager, width);
        timer.toc("getCommonKeys");
		std::cout << "Common keys: " << commonKeys.size() << std::endl;
		
        const auto &srcTable = srcTableManager->hashTable_;
        const auto &tgtTable = tgtTableManager->hashTable_;
	
		geomSetsCorrespondences.reserve(commonKeys.size());
        int srcTriIndex = 0;
        int i = 0;
        for (const auto &[key, tgtKey, srcKeyIndex]: commonKeys) {
            const auto &srcGeomSets = srcTable.at(key);
            const auto &tgtGeomSets = tgtTable.at(tgtKey);
            for (const auto &srcGeomSet: srcGeomSets) {
                for (const auto &tgtGeomSet: tgtGeomSets) {
                    geomSetsCorrespondences.emplace_back(srcGeomSet, tgtGeomSet, srcKeyIndex * 100 + srcTriIndex);
                }
                // the second time of the loop srcTriIndex++
                srcTriIndex++;
            }
            i++;
            srcTriIndex = 0;
        }
    }

    void solveCorrespondences(CorresWithID &geomSetsCorrespondences, Results2d &results) {
        results.resize(geomSetsCorrespondences.size());
#pragma omp parallel for default(none) shared(geomSetsCorrespondences, results)
        for (int i = 0; i < geomSetsCorrespondences.size(); i++) {
            const auto &geomSetCorrespondence = geomSetsCorrespondences[i];
            const Eigen::Matrix<double, 3, 2> &source = std::get<0>(geomSetCorrespondence);
            const Eigen::Matrix<double, 3, 2> &target = std::get<1>(geomSetCorrespondence);
            results[i] = solve2dClosedForm(source, target);
        }
    }
}