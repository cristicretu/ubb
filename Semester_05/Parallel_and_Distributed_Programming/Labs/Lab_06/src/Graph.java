package src;

import java.util.*;

public class Graph {
    public final int n;
    public final List<List<Integer>> adj;

    public Graph(int n) {
        this.n = n;
        this.adj = new ArrayList<>(n);
        for (int i = 0; i < n; i++) adj.add(new ArrayList<>());
    }

    public void addEdge(int u, int v) {
        adj.get(u).add(v);
    }

    public static Graph random(int n, double density, Random rng) {
        Graph g = new Graph(n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && rng.nextDouble() < density) {
                    g.addEdge(i, j);
                }
            }
        }
        return g;
    }

    public static Graph withHamiltonianCycle(int n, double extraDensity, Random rng) {
        Graph g = new Graph(n);
        int[] perm = new int[n];
        for (int i = 0; i < n; i++) perm[i] = i;
        for (int i = n - 1; i > 0; i--) {
            int j = rng.nextInt(i + 1);
            int tmp = perm[i]; perm[i] = perm[j]; perm[j] = tmp;
        }
        for (int i = 0; i < n; i++) {
            g.addEdge(perm[i], perm[(i + 1) % n]);
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && rng.nextDouble() < extraDensity) {
                    if (!g.adj.get(i).contains(j)) g.addEdge(i, j);
                }
            }
        }
        return g;
    }


    public static Graph hard(int n, Random rng) {
        Graph g = new Graph(n);
        for (int i = 0; i < n; i++) {
            g.addEdge(i, (i + 1) % n);
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && (i + 1) % n != j && rng.nextDouble() < 0.3) {
                    if (!g.adj.get(i).contains(j)) g.addEdge(i, j);
                }
            }
        }
        for (int i = 0; i < n; i++) {
            int next = (i + 1) % n;
            g.adj.get(i).remove(Integer.valueOf(next));
            Collections.shuffle(g.adj.get(i), rng);
            g.adj.get(i).add(next);
        }
        return g;
    }

    public static Graph noCycle(int n, double density, Random rng) {
        Graph g = new Graph(n);
        for (int i = 0; i < n - 1; i++) {
            g.addEdge(i, i + 1);
        }
        for (int i = 0; i < n; i++) {
            for (int j = 1; j < n; j++) {
                if (i != j && rng.nextDouble() < density) {
                    if (!g.adj.get(i).contains(j)) g.addEdge(i, j);
                }
            }
        }
        for (int i = 0; i < n; i++) {
            Collections.shuffle(g.adj.get(i), rng);
        }
        return g;
    }

    public static Graph withCycle(int n, double density, Random rng) {
        Graph g = new Graph(n);
        for (int i = 0; i < n; i++) {
            g.addEdge(i, (i + 1) % n);
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j && rng.nextDouble() < density) {
                    if (!g.adj.get(i).contains(j)) g.addEdge(i, j);
                }
            }
        }
        for (int i = 0; i < n; i++) {
            int next = (i + 1) % n;
            g.adj.get(i).remove(Integer.valueOf(next));
            Collections.shuffle(g.adj.get(i), rng);
            g.adj.get(i).add(next);
        }
        return g;
    }
}

