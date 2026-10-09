package src;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class HamiltonianParallel {
    private final Graph g;
    private final int start;
    private final int numThreads;
    private final AtomicReference<List<Integer>> result = new AtomicReference<>(null);
    private final AtomicBoolean found = new AtomicBoolean(false);

    public HamiltonianParallel(Graph g, int start, int numThreads) {
        this.g = g;
        this.start = start;
        this.numThreads = numThreads;
    }

    public List<Integer> solve() throws InterruptedException {
        boolean[] visited = new boolean[g.n];
        visited[start] = true;
        List<Integer> path = new ArrayList<>();
        path.add(start);

        List<Integer> neighbors = new ArrayList<>();
        for (int next : g.adj.get(start)) {
            if (!visited[next]) neighbors.add(next);
        }

        if (neighbors.isEmpty()) return null;

        int[] threadAlloc = allocateThreads(neighbors.size(), numThreads);
        List<Thread> threads = new ArrayList<>();

        for (int i = 0; i < neighbors.size(); i++) {
            int next = neighbors.get(i);
            int allocated = threadAlloc[i];
            boolean[] visitedCopy = visited.clone();
            visitedCopy[next] = true;
            List<Integer> pathCopy = new ArrayList<>(path);
            pathCopy.add(next);

            Thread t = new Thread(() -> search(pathCopy, visitedCopy, allocated));
            threads.add(t);
            t.start();
        }

        for (Thread t : threads) t.join();
        return result.get();
    }

    private void search(List<Integer> path, boolean[] visited, int threads) {
        if (found.get()) return;

        if (path.size() == g.n) {
            int last = path.get(path.size() - 1);
            if (g.adj.get(last).contains(start)) {
                if (found.compareAndSet(false, true)) {
                    result.set(new ArrayList<>(path));
                }
            }
            return;
        }

        int curr = path.get(path.size() - 1);
        List<Integer> neighbors = new ArrayList<>();
        for (int next : g.adj.get(curr)) {
            if (!visited[next]) neighbors.add(next);
        }

        if (neighbors.isEmpty()) return;

        if (threads <= 1) {
            for (int next : neighbors) {
                if (found.get()) return;
                visited[next] = true;
                path.add(next);
                search(path, visited, 1);
                path.remove(path.size() - 1);
                visited[next] = false;
            }
        } else {
            int[] threadAlloc = allocateThreads(neighbors.size(), threads);
            List<Thread> children = new ArrayList<>();

            for (int i = 0; i < neighbors.size(); i++) {
                int next = neighbors.get(i);
                int allocated = threadAlloc[i];
                boolean[] visitedCopy = visited.clone();
                visitedCopy[next] = true;
                List<Integer> pathCopy = new ArrayList<>(path);
                pathCopy.add(next);

                Thread t = new Thread(() -> search(pathCopy, visitedCopy, allocated));
                children.add(t);
                t.start();
            }

            for (Thread t : children) {
                try { t.join(); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            }
        }
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

