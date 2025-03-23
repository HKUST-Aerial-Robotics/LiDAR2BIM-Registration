/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef BIMREG_CLUSTERING_UTILS_H
#define BIMREG_CLUSTERING_UTILS_H

#include <vector>

class UnionFind {
public:
	UnionFind(int n);
	void unionSets(int a, int b);
	int find(int a);

private:
	std::vector<int> parent;
	std::vector<int> rank;
};

#endif //BIMREG_CLUSTERING_UTILS_H
