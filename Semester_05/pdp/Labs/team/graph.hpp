#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <random>

struct Graph {
    int n;
    std::vector<std::vector<int>> adj;

    Graph(int n) : n(n), adj(n) {}

    void add_edge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    static Graph from_file(const std::string& path) {
        std::ifstream f(path);
        int n, m;
        f >> n >> m;
        Graph g(n);
        for (int i = 0; i < m; i++) {
            int u, v;
            f >> u >> v;
            g.add_edge(u, v);
        }
        return g;
    }

    static Graph random(int n, double edge_prob, int seed = 42) {
        Graph g(n);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<> dist(0.0, 1.0);
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (dist(rng) < edge_prob) {
                    g.add_edge(i, j);
                }
            }
        }
        return g;
    }

    static Graph petersen() {
        Graph g(10);
        for (int i = 0; i < 5; i++) {
            g.add_edge(i, (i + 1) % 5);
            g.add_edge(i + 5, ((i + 2) % 5) + 5);
            g.add_edge(i, i + 5);
        }
        return g;
    }
};
