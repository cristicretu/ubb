package src;

import java.util.*;

public class HamiltonianSequential {
    private final Graph g;
    private final int start;

    public HamiltonianSequential(Graph g, int start) {
        this.g = g;
        this.start = start;
    }

    public List<Integer> solve() {
        boolean[] visited = new boolean[g.n];
        List<Integer> path = new ArrayList<>();
        path.add(start);
        visited[start] = true;
        if (dfs(path, visited)) return path;
        return null;
    }

    private boolean dfs(List<Integer> path, boolean[] visited) {
        if (path.size() == g.n) {
            int last = path.get(path.size() - 1);
            return g.adj.get(last).contains(start);
        }
        int curr = path.get(path.size() - 1);
        for (int next : g.adj.get(curr)) {
            if (!visited[next]) {
                visited[next] = true;
                path.add(next);
                if (dfs(path, visited)) return true;
                path.remove(path.size() - 1);
                visited[next] = false;
            }
        }
        return false;
    }
}

