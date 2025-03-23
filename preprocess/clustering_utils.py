class UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n

    def union(self, a, b):
        root_a = self.find(a)
        root_b = self.find(b)
        if root_a != root_b:
            if self.rank[root_a] > self.rank[root_b]:
                self.parent[root_b] = root_a
            else:
                self.parent[root_a] = root_b
                if self.rank[root_a] == self.rank[root_b]:
                    self.rank[root_b] += 1

    def find(self, a):
        if self.parent[a] != a:
            self.parent[a] = self.find(self.parent[a])
        return self.parent[a]
