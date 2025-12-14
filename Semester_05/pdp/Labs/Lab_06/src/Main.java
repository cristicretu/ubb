package src;

import java.util.*;

public class Main {
    public static void main(String[] args) throws Exception {
        Scanner sc = new Scanner(System.in);
        System.out.print("nodes: "); int n = sc.nextInt();
        System.out.print("edges: "); int edges = sc.nextInt();
        System.out.print("has cycle (1/0): "); boolean hasCycle = sc.nextInt() == 1;
        sc.close();

        double density = (double) edges / (n * (n - 1));
        Graph g = hasCycle ? Graph.withCycle(n, density, new Random(42)) : Graph.noCycle(n, density, new Random(42));
        int actualEdges = g.adj.stream().mapToInt(List::size).sum();
        System.out.printf("\nGraph: n=%d, edges=%d, cycle=%s\n", n, actualEdges, hasCycle);

        int[] threadCounts = {1, 2, 4, 8};

        long t0 = System.nanoTime();
        List<Integer> seqResult = new HamiltonianSequential(g, 0).solve();
        long seqTime = System.nanoTime() - t0;
        System.out.printf("  Sequential: %.3f ms %s\n", seqTime / 1e6, seqResult != null ? "FOUND" : "NONE");

        for (int threads : threadCounts) {
            t0 = System.nanoTime();
            List<Integer> parResult = new HamiltonianParallel(g, 0, threads).solve();
            long parTime = System.nanoTime() - t0;
            System.out.printf("  Parallel(%d): %.3f ms %s (speedup: %.2fx)\n",
                threads, parTime / 1e6, parResult != null ? "FOUND" : "NONE", (double)seqTime / parTime);

            t0 = System.nanoTime();
            List<Integer> fjResult = new HamiltonianForkJoin(g, 0, threads).solve();
            long fjTime = System.nanoTime() - t0;
            System.out.printf("  ForkJoin(%d): %.3f ms %s (speedup: %.2fx)\n",
                threads, fjTime / 1e6, fjResult != null ? "FOUND" : "NONE", (double)seqTime / fjTime);
        }
    }

    static boolean verify(Graph g, List<Integer> path) {
        if (path.size() != g.n) return false;
        Set<Integer> seen = new HashSet<>(path);
        if (seen.size() != g.n) return false;
        for (int i = 0; i < path.size(); i++) {
            int from = path.get(i);
            int to = path.get((i + 1) % path.size());
            if (!g.adj.get(from).contains(to)) return false;
        }
        return true;
    }
}

