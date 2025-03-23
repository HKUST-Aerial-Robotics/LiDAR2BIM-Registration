#ifndef BIMREG_HOUGH_VOTING_H
#define BIMREG_HOUGH_VOTING_H

#include <Eigen/Core>
#include "frontend/feature/match.h"
#include "backend/hough/pose_2d.h"
#include "utils/cfg.h"
#include "utils/clustering_utils.h"
#include <nanoflann.hpp>
#include "utils/nanoflann_utils.h"
#include "utils/utils.h"
#include <random>
#include <algorithm>
#include <pcl/visualization/pcl_visualizer.h>
namespace hough {

    struct Array3D {
        Eigen::Array<int, 3, 1> data;

        Array3D(const int &x, const int &y, const int &z) : data(x, y, z) {}

        bool operator==(const Array3D &other) const {
            return (data == other.data).all();
        }

        bool operator!=(const Array3D &other) {
            return !(*this == other);
        }

        int operator[](int i) const {
            return data[i];
        }

        int &operator[](int i) {
            return data[i];
        }

        Array3D &operator=(const Array3D &other) {
            data = other.data;
            return *this;
        }

        Array3D() : data(Eigen::Array<int, 3, 1>::Zero()) {}

        std::string toString() const {
            std::string str = "";
            for (int i = 0; i < data.size(); ++i) {
                str += std::to_string(data[i]) + " ";
            }
            return str;
        }

    };

    struct Array3DHash {
        std::uint64_t operator()(const Array3D &a) const {
            std::uint64_t hash = 0;
            int s1 = 1;
            int s2 = s1 * 2000;
            int s3 = s2 * 2000;
            hash = (a[0] + 300) * s1 + (a[1] + 300) * s2 + (a[2] + 300) * s3;
            return hash;

        }
    };

    using Corres = std::vector<std::pair<Eigen::Matrix<double, 3, 2>, Eigen::Matrix<double, 3, 2>>>;
    using VoteKey = Array3D;
    using tripletIndex = int;
    using PoseList = std::vector<Pose2D>;
    using kdtree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
            PointCloud<double>, 3 /* dim */>;

    class HoughVote {
    public:


        HoughVote(double xyRes, double yawRes);
		HoughVote();

        std::vector<Eigen::Vector3d>
        vote(const geomset::CorresWithID &geomSetsCorrespondences, const geomset::Results2d &results, int width,
             bool mergeNeigh = true);

        Array3D tfToVoteKey(const std::tuple<double, double, double> &tf);
	
		std::vector<Eigen::Vector3d> keysToCandidates(const std::vector<VoteKey> &keys);
	
		std::vector<Eigen::Vector3d> keysToCandidatesSVD(const std::vector<VoteKey> &keys);
	
		void initHoughSpace(const geomset::CorresWithID &geomSetsCorrespondences, const geomset::Results2d &results);

        void getTopJKeys(int width, bool mergeNeigh = true);

        void truncateKeysTopL();

        void mergeNeighVoxel(int width = 1);

        void truncateKeysTopK();

        void initKDTree();

        void getNMStopKKeys();
		
        void KDTreeUF();

        void truncateKeysTopJ();

        double unionArea(const VoteKey &key);
		
		int rawHoughSize = 0;
		std::unordered_map<VoteKey, std::unordered_map<tripletIndex, PoseList>, Array3DHash> getHoughSpace(){return houghSpace_;}
		std::unordered_map<int, std::vector<Array3D>> getClusters(){return clusters;}
		std::vector<VoteKey> getTopJKeys(){return topJKeys;}
		std::vector<VoteKey> getTopKKeys(){return topKKeys;}
		std::vector<VoteKey> fetchNMStopKKeys(){return NMStopKKeys;}


    protected:

        double xyRes_;
        double yawRes_;
        std::unordered_map<VoteKey, std::unordered_map<tripletIndex, PoseList>, Array3DHash> houghSpace_;
		std::unordered_set<VoteKey, Array3DHash> topLKeys;
        std::vector<VoteKey> topJKeys;
        std::vector<VoteKey> topKKeys;
        std::vector<VoteKey> NMStopKKeys;
        std::function<int(const Array3D &)> topLScoreFunc;
        std::function<int(const Array3D &)> topKScoreFunc;
        std::function<int(const Array3D &)> topJScoreFunc;
        UnionFind uf = UnionFind(1);
        PointCloud<double> keysCloud;
        std::shared_ptr<kdtree> treePtr;
		std::unordered_map<int, std::vector<Array3D>> clusters;

        std::unordered_map<Array3D, double, Array3DHash> scoreCache;
    };
}

#endif //BIMREG_HOUGH_VOTING_H
