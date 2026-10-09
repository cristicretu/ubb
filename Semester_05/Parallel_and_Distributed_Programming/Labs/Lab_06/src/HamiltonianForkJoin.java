package src;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class HamiltonianForkJoin {
    private final Graph g;
    private final int start;
    private final ForkJoinPool pool;
    private final AtomicBoolean found = new AtomicBoolean(false);

    public HamiltonianForkJoin(Graph g, int start, int numThreads) {
        this.g = g;
        this.start = start;
        this.pool = new ForkJoinPool(numThreads);
    }

    public List<Integer> solve() {
        boolean[] visited = new boolean[g.n];
        visited[start] = true;
        List<Integer> path = new ArrayList<>();
        path.add(start);

        try {
            return pool.invoke(new HamiltonianTask(path, visited, pool.getParallelism()));
        } finally {
            pool.shutdown();
        }
    }

    private class HamiltonianTask extends RecursiveTask<List<Integer>> {
        private final List<Integer> path;
        private final boolean[] visited;
        private final int threads;

        HamiltonianTask(List<Integer> path, boolean[] visited, int threads) {
            this.path = path;
            this.visited = visited;
            this.threads = threads;
        }

        @Override
        protected List<Integer> compute() {
            if (found.get()) return null;

            if (path.size() == g.n) {
                int last = path.get(path.size() - 1);
                if (g.adj.get(last).contains(start)) {
                    found.set(true);
                    return new ArrayList<>(path);
                }
                return null;
            }

            int curr = path.get(path.size() - 1);
            List<Integer> neighbors = new ArrayList<>();
            for (int next : g.adj.get(curr)) {
                if (!visited[next]) neighbors.add(next);
            }

            if (neighbors.isEmpty()) return null;

            if (threads <= 1) {
                return sequential();
            }

            int[] threadAlloc = allocateThreads(neighbors.size(), threads);
            List<HamiltonianTask> tasks = new ArrayList<>();

            for (int i = 0; i < neighbors.size(); i++) {
                int next = neighbors.get(i);
                boolean[] visitedCopy = visited.clone();
                visitedCopy[next] = true;
                List<Integer> pathCopy = new ArrayList<>(path);
                pathCopy.add(next);
                tasks.add(new HamiltonianTask(pathCopy, visitedCopy, threadAlloc[i]));
            }

            for (int i = 1; i < tasks.size(); i++) {
                tasks.get(i).fork();
            }

            List<Integer> result = tasks.get(0).compute();
            if (result != null) return result;

            for (int i = 1; i < tasks.size(); i++) {
                List<Integer> r = tasks.get(i).join();
                if (r != null) return r;
            }

            return null;
        }

        private List<Integer> sequential() {
            if (found.get()) return null;

            if (path.size() == g.n) {
                int last = path.get(path.size() - 1);
                if (g.adj.get(last).contains(start)) {
                    found.set(true);
                    return new ArrayList<>(path);
                }
                return null;
            }

            int curr = path.get(path.size() - 1);
            for (int next : g.adj.get(curr)) {
                if (!visited[next]) {
                    visited[next] = true;
                    path.add(next);
                    List<Integer> result = sequential();
                    if (result != null) return result;
                    path.remove(path.size() - 1);
                    visited[next] = false;
                }
            }
            return null;
        }

        private int[] allocateThreads(int branches, int threads) {
            int[] alloc = new int[branches];
            int base = threads / branches;
            int extra = threads % branches;
            for (int i = 0; i < branches; i++) {
                alloc[i] = base + (i < extra ? 1 : 0);
            }
            return alloc;
        }
    }
}

