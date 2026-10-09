#include "parallel_coloring.hpp"
#include <chrono>
#include <cstdio>

using Clock = std::chrono::high_resolution_clock;

template<typename F>
double benchmark(F&& f) {
    auto start = Clock::now();
    f();
    auto end = Clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void run_test(const std::string& name, const Graph& g, int k) {
    printf("\n=== %s (n=%d, k=%d) ===\n", name.c_str(), g.n, k);

    int hw_threads = std::thread::hardware_concurrency();
    printf("hardware threads: %d\n", hw_threads);

    std::optional<Coloring> seq_result;
    double seq_time = benchmark([&] {
        seq_result = solve_sequential(g, k);
    });
    printf("sequential:   %.2f ms %s\n", seq_time,
           seq_result ? "(found)" : "(no solution)");

    for (int n_threads : {2, 4, 8}) {
        if (n_threads > hw_threads * 2) continue;

        std::optional<Coloring> par_result;
        double par_time = benchmark([&] {
            par_result = solve_parallel(g, k, n_threads);
        });
        printf("parallel(%d):  %.2f ms (speedup: %.2fx)\n",
               n_threads, par_time, seq_time / par_time);

        if (par_result && !par_result->valid(g)) {
            printf("ERROR: invalid coloring!\n");
        }
    }

    for (int n_threads : {2, 4, 8}) {
        if (n_threads > hw_threads * 2) continue;

        std::optional<Coloring> pool_result;
        double pool_time = benchmark([&] {
            pool_result = solve_threadpool(g, k, n_threads);
        });
        printf("threadpool(%d): %.2f ms (speedup: %.2fx)\n",
               n_threads, pool_time, seq_time / pool_time);

        if (pool_result && !pool_result->valid(g)) {
            printf("ERROR: invalid coloring!\n");
        }
    }
}

int main(int argc, char** argv) {
    if (argc > 1) {
        std::string path = argv[1];
        int k = argc > 2 ? std::stoi(argv[2]) : 3;
        Graph g = Graph::from_file(path);
        run_test("custom graph", g, k);
        return 0;
    }

    run_test("petersen graph", Graph::petersen(), 3);
    run_test("random small", Graph::random(20, 0.3), 4);
    run_test("random medium", Graph::random(25, 0.4), 5);
    run_test("random dense", Graph::random(22, 0.6), 6);

    printf("\n--- detailed output for petersen ---\n");
    auto g = Graph::petersen();
    auto result = solve_parallel(g, 3, 4);
    if (result) {
        result->print();
        printf("valid: %s\n", result->valid(g) ? "yes" : "no");
    }

    return 0;
}
