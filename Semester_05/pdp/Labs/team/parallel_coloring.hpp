#pragma once
#include "coloring.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>

struct ParallelColoring {
    const Graph& g;
    int k;
    int n_threads;
    std::atomic<bool> found{false};
    std::mutex result_mutex;
    std::optional<Coloring> result;

    ParallelColoring(const Graph& g, int k, int n_threads)
        : g(g), k(k), n_threads(n_threads) {}

    bool backtrack(std::vector<int>& colors, int node) {
        if (found.load(std::memory_order_relaxed)) return false;
        if (node == g.n) return true;

        for (int c = 0; c < k && !found.load(std::memory_order_relaxed); c++) {
            if (is_safe(g, colors, node, c)) {
                colors[node] = c;
                if (backtrack(colors, node + 1)) return true;
                colors[node] = -1;
            }
        }
        return false;
    }

    void worker(std::vector<int> initial_colors, int start_node) {
        if (backtrack(initial_colors, start_node)) {
            std::lock_guard<std::mutex> lock(result_mutex);
            if (!found.exchange(true)) {
                result = Coloring{initial_colors, k};
            }
        }
    }

    std::optional<Coloring> solve() {
        if (g.n == 0) return Coloring{{}, k};

        std::vector<std::pair<std::vector<int>, int>> tasks;
        std::queue<std::pair<std::vector<int>, int>> q;
        q.push({std::vector<int>(g.n, -1), 0});

        int target_tasks = n_threads * 4;
        while (!q.empty() && (int)tasks.size() < target_tasks) {
            auto [colors, node] = q.front();
            q.pop();

            if (node == g.n) {
                return Coloring{colors, k};
            }

            bool expanded = false;
            for (int c = 0; c < k; c++) {
                if (is_safe(g, colors, node, c)) {
                    auto new_colors = colors;
                    new_colors[node] = c;
                    if ((int)(tasks.size() + q.size()) < target_tasks && node < g.n / 3) {
                        q.push({new_colors, node + 1});
                        expanded = true;
                    } else {
                        tasks.push_back({new_colors, node + 1});
                    }
                }
            }
            if (!expanded && tasks.empty() && q.empty()) {
                return std::nullopt;
            }
        }

        while (!q.empty()) {
            tasks.push_back(q.front());
            q.pop();
        }

        if (tasks.empty()) return std::nullopt;

        std::vector<std::thread> threads;
        for (auto& [colors, start] : tasks) {
            threads.emplace_back(&ParallelColoring::worker, this, colors, start);
        }

        for (auto& t : threads) t.join();
        return result;
    }
};

struct ThreadPoolColoring {
    const Graph& g;
    int k;
    int n_threads;
    std::atomic<bool> found{false};
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<std::pair<std::vector<int>, int>> work_queue;
    std::optional<Coloring> result;
    std::atomic<int> active_workers{0};
    bool shutdown{false};

    ThreadPoolColoring(const Graph& g, int k, int n_threads)
        : g(g), k(k), n_threads(n_threads) {}

    void worker_loop() {
        while (true) {
            std::pair<std::vector<int>, int> task;
            {
                std::unique_lock<std::mutex> lock(mtx);
                cv.wait(lock, [this] {
                    return !work_queue.empty() || shutdown || found.load();
                });

                if ((shutdown && work_queue.empty()) || found.load()) return;
                task = std::move(work_queue.front());
                work_queue.pop();
                active_workers++;
            }

            auto& [colors, node] = task;
            backtrack_with_split(colors, node, 0);

            {
                std::lock_guard<std::mutex> lock(mtx);
                active_workers--;
                if (work_queue.empty() && active_workers == 0) {
                    shutdown = true;
                    cv.notify_all();
                }
            }
        }
    }

    void backtrack_with_split(std::vector<int>& colors, int node, int depth) {
        if (found.load(std::memory_order_relaxed)) return;
        if (node == g.n) {
            std::lock_guard<std::mutex> lock(mtx);
            if (!found.exchange(true)) {
                result = Coloring{colors, k};
            }
            cv.notify_all();
            return;
        }

        bool should_split = depth < 2 && node < g.n / 2;

        for (int c = 0; c < k && !found.load(std::memory_order_relaxed); c++) {
            if (is_safe(g, colors, node, c)) {
                colors[node] = c;

                if (should_split) {
                    std::lock_guard<std::mutex> lock(mtx);
                    work_queue.push({colors, node + 1});
                    cv.notify_one();
                } else {
                    backtrack_with_split(colors, node + 1, depth + 1);
                }

                colors[node] = -1;
            }
        }
    }

    std::optional<Coloring> solve() {
        if (g.n == 0) return Coloring{{}, k};

        work_queue.push({std::vector<int>(g.n, -1), 0});

        std::vector<std::thread> threads;
        for (int i = 0; i < n_threads; i++) {
            threads.emplace_back(&ThreadPoolColoring::worker_loop, this);
        }

        for (auto& t : threads) t.join();
        return result;
    }
};

inline std::optional<Coloring> solve_parallel(const Graph& g, int k, int n_threads) {
    ParallelColoring solver(g, k, n_threads);
    return solver.solve();
}

inline std::optional<Coloring> solve_threadpool(const Graph& g, int k, int n_threads) {
    ThreadPoolColoring solver(g, k, n_threads);
    return solver.solve();
}
