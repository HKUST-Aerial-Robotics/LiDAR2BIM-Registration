#include "utils/clustering_utils.h"
UnionFind::UnionFind(int n) {
	parent.resize(n);
	rank.resize(n, 0);
	for (int i = 0; i < n; ++i) {
		parent[i] = i;
	}
}

void UnionFind::unionSets(int a, int b) {
	int rootA = find(a);
	int rootB = find(b);
	if (rootA != rootB) {
		if (rank[rootA] > rank[rootB]) {
			parent[rootB] = rootA;
		} else {
			parent[rootA] = rootB;
			if (rank[rootA] == rank[rootB]) {
				rank[rootB] += 1;
			}
		}
	}
}

int UnionFind::find(int a) {
	if (parent[a] != a) {
		parent[a] = find(parent[a]);
	}
	return parent[a];
}