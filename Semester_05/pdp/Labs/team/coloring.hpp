#pragma once
#include "graph.hpp"
#include <atomic>
#include <optional>

struct Coloring {
    std::vector<int> colors;
    int n_colors;

    bool valid(const Graph& g) const {
        for (int u = 0; u < g.n; u++) {
            if (colors[u] < 0 || colors[u] >= n_colors) return false;
            for (int v : g.adj[u]) {
                if (colors[u] == colors[v]) return false;
            }
        }
        return true;
    }

    void print() const {
        for (int i = 0; i < (int)colors.size(); i++) {
            printf("node %d -> color %d\n", i, colors[i]);
        }
    }
};

inline bool is_safe(const Graph& g, const std::vector<int>& colors, int node, int color) {
    for (int neighbor : g.adj[node]) {
        if (colors[neighbor] == color) return false;
    }
    return true;
}

inline std::optional<Coloring> solve_sequential(const Graph& g, int k) {
    std::vector<int> colors(g.n, -1);

    std::function<bool(int)> backtrack = [&](int node) -> bool {
        if (node == g.n) return true;

        for (int c = 0; c < k; c++) {
            if (is_safe(g, colors, node, c)) {
                colors[node] = c;
                if (backtrack(node + 1)) return true;
                colors[node] = -1;
            }
        }
        return false;
    };

    if (backtrack(0)) {
        return Coloring{colors, k};
    }
    return std::nullopt;
}
