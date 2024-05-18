#include <climits>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <vector>

class Graph {
 private:
  uint32_t vertices = 0, edges = 0;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> outbound;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> inbound;
  std::unordered_map<int, int> cost;

  int minCost;
  std::vector<int> bestPath;

  /// For the biconnected components
  std::vector<int> discoveryLevel, lowReach;
  std::stack<uint32_t> stack;
  std::vector<std::vector<uint32_t>> articulated_components;

  std::vector<int> parent, rank;

  int generateEdgeID() { return edges++; }

 public:
  Graph(uint32_t vertices = 0, uint32_t edges = 0)
      : vertices(vertices), edges(edges) {}

  ~Graph() {
    for (auto& edge : outbound) {
      edge.second.clear();
    }
    for (auto& edge : inbound) {
      edge.second.clear();
    }
  }

  void setVertices(uint32_t vertices) { this->vertices = vertices; }

  void setEdges(uint32_t edges) { this->edges = edges; }

  void setCost(uint32_t source, uint32_t target, int cost) {
    if (isEdge(source, target)) {
      this->cost[getEdgeID(source, target)] = cost;
    }
  }

  uint32_t getVertices() const { return vertices; }

  uint32_t getEdges() const { return edges; }

  int getCost(uint32_t source, uint32_t target) const {
    if (isEdge(source, target)) {
      int edgeID = getEdgeID(source, target);
      auto it = cost.find(edgeID);
      if (it != cost.end()) {
        return it->second;
      }
    }
    return -1;
  }

  using EdgeIterator = std::list<std::pair<uint32_t, uint32_t>>::const_iterator;

  std::pair<EdgeIterator, EdgeIterator> getOutboundEdges(
      uint32_t vertex) const {
    if (outbound.find(vertex) != outbound.end()) {
      return {outbound.at(vertex).cbegin(), outbound.at(vertex).cend()};
    }
    static const std::list<std::pair<uint32_t, uint32_t>> empty;
    return {empty.cbegin(), empty.cend()};
  }

  std::pair<EdgeIterator, EdgeIterator> getInboundEdges(uint32_t vertex) const {
    if (inbound.find(vertex) != inbound.end()) {
      return {inbound.at(vertex).cbegin(), inbound.at(vertex).cend()};
    }
    static const std::list<std::pair<uint32_t, uint32_t>> empty;
    return {empty.cbegin(), empty.cend()};
  }

  std::vector<std::pair<uint32_t, uint32_t>> getOutEdges(
      uint32_t vertex) const {
    if (outbound.find(vertex) != outbound.end()) {
      return std::vector<std::pair<uint32_t, uint32_t>>(
          outbound.at(vertex).begin(), outbound.at(vertex).end());
    }
    return std::vector<std::pair<uint32_t, uint32_t>>();
  }

  std::vector<std::pair<uint32_t, uint32_t>> getInEdges(uint32_t vertex) const {
    if (inbound.find(vertex) != inbound.end()) {
      return std::vector<std::pair<uint32_t, uint32_t>>(
          inbound.at(vertex).begin(), inbound.at(vertex).end());
    }
    return std::vector<std::pair<uint32_t, uint32_t>>();
  }

  std::vector<uint32_t> getVerticesList() const {
    std::vector<uint32_t> verticesList;
    for (auto edge : outbound) {
      verticesList.push_back(edge.first);
    }
    return verticesList;
  }

  uint32_t getEdgeID(uint32_t source, uint32_t target) const {
    if (outbound.find(source) != outbound.end()) {
      for (const auto& edge : outbound.at(source)) {
        if (edge.first == target) {
          return edge.second;
        }
      }
    }
    return -1;
  }

  bool isEdge(uint32_t source, uint32_t target) const {
    return getEdgeID(source, target) != -1;
  }

  uint32_t getInDegree(uint32_t vertex) const {
    if (inbound.find(vertex) != inbound.end()) {
      return inbound.at(vertex).size();
    }
    return 0;
  }

  uint32_t getOutDegree(uint32_t vertex) const {
    if (outbound.find(vertex) != outbound.end()) {
      return outbound.at(vertex).size();
    }
    return 0;
  }

  void addEdge(uint32_t source, uint32_t target, int cost) {
    if (!this->isEdge(source, target)) {
      int edgeID = generateEdgeID();
      this->outbound[source].push_back({target, edgeID});
      this->outbound[target].push_back({source, edgeID});
      this->cost[edgeID] = cost;
    }
  }

  void removeEdge(uint32_t source, uint32_t target) {
    if (this->isEdge(source, target)) {
      int edgeID = getEdgeID(source, target);
      outbound[source].remove_if(
          [edgeID](const std::pair<uint32_t, uint32_t>& edge) {
            return edge.second == edgeID;
          });
      inbound[target].remove_if(
          [edgeID](const std::pair<uint32_t, uint32_t>& edge) {
            return edge.second == edgeID;
          });
      cost.erase(edgeID);
    }
  }

  void addVertex(uint32_t vertex) {
    if (outbound.find(vertex) == outbound.end()) {
      outbound[vertex] = std::list<std::pair<uint32_t, uint32_t>>();
      inbound[vertex] = std::list<std::pair<uint32_t, uint32_t>>();
      vertices++;
    }
  }

  void removeVertex(uint32_t vertex) {
    if (outbound.find(vertex) != outbound.end()) {
      for (auto& edge : std::vector<std::pair<uint32_t, uint32_t>>(
               outbound[vertex].begin(), outbound[vertex].end())) {
        removeEdge(vertex, edge.first);
      }

      for (auto& edge : std::vector<std::pair<uint32_t, uint32_t>>(
               inbound[vertex].begin(), inbound[vertex].end())) {
        removeEdge(edge.first, vertex);
      }

      outbound.erase(vertex);
      inbound.erase(vertex);

      vertices--;
    }
  }

  std::pair<uint32_t, uint32_t> getEndpoints(uint32_t edgeID) const {
    for (auto& edge : outbound) {
      for (auto& target : edge.second) {
        if (target.second == edgeID) {
          return {edge.first, target.first};
        }
      }
    }
    return {-1, -1};
  }

  Graph copyGraph() const {
    Graph copy(vertices, edges);
    for (auto& edge : outbound) {
      for (auto& target : edge.second) {
        copy.addEdge(edge.first, target.first,
                     getCost(edge.first, target.first));
      }
    }

    return copy;
  }

  static Graph createRandomGraph(uint32_t vertices, uint32_t maxEdges) {
    Graph randomGraph;
    randomGraph.setVertices(vertices);

    for (uint32_t i = 0; i < maxEdges; i++) {
      uint32_t source = rand() % vertices;
      uint32_t target = rand() % vertices;

      if (source != target && !randomGraph.isEdge(source, target)) {
        int cost = rand() % 100;
        randomGraph.addEdge(source, target, cost);
      } else {
        i--;
      }
    }

    return randomGraph;
  }

  /// @brief  Find the lowest length path between two vertices using BFS
  /// @param startVertex The starting vertex
  /// @param endVertex The ending vertex
  /// @return The lowest length path between the two vertices (if it exists)
  std::vector<uint32_t> findLowestLengthPath(uint32_t startVertex,
                                             uint32_t endVertex) {
    std::unordered_map<uint32_t, uint32_t>
        distance;  /// For storing the distances from the endVertex, we use this
                   /// to mark the vertices as visited or not

    std::unordered_map<uint32_t, uint32_t>
        predecessor;  /// For storing the predecessor of each vertex, to
                      /// reconstruct the path

    std::queue<uint32_t> queue;  /// For BFS traversal

    for (const auto& p : outbound) {  /// Mark all vertices as unvisited
      distance[p.first] = UINT_MAX;
    }

    distance[endVertex] = 0;  /// Distance from endVertex to itself is 0
    queue.push(endVertex);    /// Start from the endVertex

    while (!queue.empty()) {                   /// BFS traversal
      uint32_t currentVertex = queue.front();  /// Retrieve the current vertex
      queue.pop();  /// Remove the current vertex from the queue

      for (const auto& edge :
           getInEdges(currentVertex)) {  /// Traverse through all inboung edges
                                         /// of the current vertex
        uint32_t neighbor = edge.first;
        if (distance[neighbor] ==
            UINT_MAX) {  /// If the neighbor vertex is unvisited, push it to the
                         /// queue, update the distance and predecessor
          distance[neighbor] =
              distance[currentVertex] + getCost(neighbor, currentVertex);
          predecessor[neighbor] = currentVertex;
          queue.push(neighbor);
        }
      }
    }

    std::vector<uint32_t> path;  /// Reconstruct the path from startVertex to
                                 /// endVertex using the predecessor map
    if (distance[startVertex] != UINT_MAX) {  /// This means that a path exists
      /// Go down the tree from startVertex to endVertex
      for (uint32_t at = startVertex; at != endVertex; at = predecessor[at]) {
        path.push_back(at);
      }
      /// Push also the endVertex
      path.push_back(endVertex);
    }

    return path;
  }

  /// @brief Get the transpose of the graph, by reversing the edges
  /// @return The transpose of the graph
  Graph getTranspose() const {
    Graph g(vertices);
    for (auto& vertex : outbound) {
      for (auto& pair : vertex.second) {
        g.addEdge(pair.first, vertex.first, getCost(vertex.first, pair.first));
      }
    }
    return g;
  }

  /// @brief Visit the vertex and all its out edges
  /// @param node  The current vertex
  /// @param visited  The vector of visited vertices
  /// @param finishStack  The stack of vertices (used for computing the strongly
  /// connected components)
  void DFS_Outbound(uint32_t node, std::vector<bool>& visited,
                    std::stack<uint32_t>& finishStack) {
    visited[node] = true;
    auto edges = this->getOutEdges(node);  /// Iterate over out edges
    for (const auto& edge : edges) {  /// If the vertex is not visited, visit it
      if (!visited[edge.first]) {
        DFS_Outbound(edge.first, visited, finishStack);
      }
    }
    finishStack.push(node);  /// Push the vertex to the stack
  }

  /// @brief Visit the vertex and all its in edges
  /// @param node  The current vertex
  /// @param visited  The vector of visited vertices
  /// @param component  The vector of the current strongly connected component
  void DFS_Inbound(uint32_t node, std::vector<bool>& visited,
                   std::vector<uint32_t>& component) {
    visited[node] = true;
    component.push_back(node);
    auto edges = this->getInEdges(node);  /// Iterate over in edges
    for (const auto& edge : edges) {
      if (!visited[edge.first]) {
        DFS_Inbound(edge.first, visited, component);
      }
    }
  }

  /// @brief Find the strongly connected components of the graph
  /// @return The vector of strongly connected components
  std::vector<std::vector<uint32_t>> findStronglyConnectedComponents() {
    std::vector<bool> visited(
        this->vertices, false);  /// Initialize all vertices as not visited
    std::stack<uint32_t> finishStack;  /// Stack to store the finishing times
    std::vector<std::vector<uint32_t>>
        stronglyConnectedComponents;  /// Store the strongly connected
                                      /// components

    // Fill vertices in stack according to their finishing times
    for (uint32_t i = 0; i < this->vertices; ++i) {
      if (!visited[i]) {
        DFS_Outbound(i, visited, finishStack);
      }
    }

    // Mark all the vertices as not visited (for the second DFS)
    std::fill(visited.begin(), visited.end(), false);

    while (!finishStack.empty()) {
      uint32_t v = finishStack.top();
      finishStack.pop();

      // Get one strongly connected component of the popped vertex
      if (!visited[v]) {
        std::vector<uint32_t> component;
        this->DFS_Inbound(v, visited, component);
        stronglyConnectedComponents.push_back(component);
      }
    }

    std::cout << "Size of strongly connected components: "
              << stronglyConnectedComponents.size() << "\n";
    return stronglyConnectedComponents;
  }

  /* --------------------------------------------------------------- */

  std::vector<int> getPath(const std::vector<std::vector<int>>& path,
                           int source, int target) {
    if (path[source][target] == -1) return {};  // No path exists
    std::vector<int> result = {source};
    while (source != target && path[source][target] != source) {
      source = path[source][target];
      result.push_back(source);
    }
    result.push_back(target);
    return result;
  }

  /// @brief Matrix multiplication
  /// @param A  The first matrix
  /// @param B  The second matrix
  /// @param V The number of vertices
  /// @return The resulting matrix C = A * B
  std::vector<std::vector<int>> multiplyMatrixWithPath(
      const std::vector<std::vector<int>>& A,
      const std::vector<std::vector<int>>& B, int V,
      std::vector<std::vector<int>>& path) {
    std::vector<std::vector<int>> C(V, std::vector<int>(V, INT_MAX));
    for (int i = 0; i < V; ++i) {
      for (int j = 0; j < V; ++j) {
        for (int k = 0; k < V; ++k) {
          if (A[i][k] < INT_MAX && B[k][j] < INT_MAX) {
            int possibleCost = A[i][k] + B[k][j];
            if (possibleCost < C[i][j]) {
              C[i][j] = possibleCost;
              path[i][j] = k;
            }
          }
        }
      }
    }
    return C;
  }

  std::vector<std::vector<int>> multiplyMatrix(
      const std::vector<std::vector<int>>& A,
      const std::vector<std::vector<int>>& B, int V) {
    std::vector<std::vector<int>> C(V, std::vector<int>(V, INT_MAX));
    for (int i = 0; i < V; ++i) {
      for (int j = 0; j < V; ++j) {
        for (int k = 0; k < V; ++k) {
          if (A[i][k] < INT_MAX && B[k][j] < INT_MAX) {
            int possibleCost = A[i][k] + B[k][j];
            if (possibleCost < C[i][j]) {
              C[i][j] = possibleCost;
            }
          }
        }
      }
    }
    return C;
  }

  /// @brief Detect a negative cycle in the graph, using the A^V matrix
  /// @param W The weight adjacency matrix
  /// @param V The number of vertices
  /// @return True if a negative cycle is detected, false otherwise
  bool detectNegativeCycle(const std::vector<std::vector<int>>& W, int V) {
    /*
    Following the observation that for an adjacency matrix, A^n has the property
    that the entry row i, column j is the number of walks of length n from i to
    j.

    This means that if the matrix A^n contains at least one non-zero entry on
    the diagonal, then there is a cycle of negative weight.
    */
    std::vector<std::vector<int>> M = W;  /// Initialize the matrix M with W
    for (int m = 1; m < V; ++m) {
      M = multiplyMatrix(M, W,
                         V);  /// Compute the matrix M = W^m for m = 1 to V
      for (int i = 0; i < V;
           ++i) {  /// Check the trace of the matrix M for negative values
        if (M[i][i] < 0) {
          return true;  // Negative cycle detected
        }
      }
    }
    return false;
  }

  void findLowestCostWalk(uint32_t source, uint32_t target) {
    uint32_t V = this->getVertices();
    std::vector<std::vector<int>> W(
        V,
        std::vector<int>(V, INT_MAX));  /// Create the weight adjacency matrix
    std::vector<std::vector<int>> path(V, std::vector<int>(V, -1));

    for (uint32_t i = 0; i < V; ++i) {
      for (const auto& edge : this->getOutEdges(i)) {
        W[i][edge.first] =
            this->getCost(i, edge.first);  /// Fill the matrix with the costs
        path[i][edge.first] = i;           /// Initialize the path
      }
      W[i][i] = 0;
      path[i][i] = i;
    }

    if (detectNegativeCycle(W, V)) {
      std::cout << "The graph contains a negative cost cycle.\n";
      return;
    }

    std::vector<std::vector<int>> D =
        W;                         /// Initialize the distance matrix, has
                                   /// the same values as the weight matrix
    for (int m = 1; m < V; ++m) {  /// Iterate over the vertices
      D = multiplyMatrixWithPath(D, W, V,
                                 path);  /// Multiply the distance matrix
                                         /// with the weight matrix
    }

    if (D[source][target] == INT_MAX) {  /// If the cost is INT_MAX, there is no
                                         /// path between the source and target
      std::cout << "No path exists from " << source << " to " << target
                << ".\n";
    } else {  /// Otherwise, output the lowest cost
      std::cout << "The lowest cost from " << source << " to " << target
                << " is " << D[source][target] << ".\n";

      // Construct the path

      std::vector<int> result = getPath(path, source, target);
      std::cout << "Path: ";
      for (int i = 0; i < result.size(); ++i) {
        std::cout << result[i];
        if (i < result.size() - 1) {
          std::cout << " -> ";
        }
      }
    }
  }

  /// @brief Count the number of distinct walks of minimum cost between two
  /// vertices
  /// @param start The starting vertex
  /// @return A vector of the minimum costs from the starting vertex to all
  /// other vertices
  std::vector<int> dijkstra(uint32_t start) const {
    std::vector<int> dist(vertices,
                          INT_MAX);  /// Initialize the distance vector
    dist[start] = 0;  /// The distance from the starting vertex to itself is 0
    using pii = std::pair<int, uint32_t>;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>>
        pq;               /// Min heap for the vertices
    pq.push({0, start});  /// Push the starting vertex to the heap

    while (!pq.empty()) {
      auto [cost, u] = pq.top();  /// Retrieve the vertex with the lowest cost
      pq.pop();
      if (cost > dist[u]) continue;  /// If the cost is higher, skip the vertex

      auto it_range = getOutboundEdges(u);  /// Iterate over the outbound edges
      for (auto it = it_range.first; it != it_range.second;
           ++it) {               /// Relax the edges
        auto [v, weight] = *it;  /// Retrieve the target vertex and the weight
        int new_cost = cost + getCost(u, v);  /// Calculate the new cost
        if (new_cost < dist[v]) {  /// If the new cost is lower, update the
                                   /// distance and push the vertex to the heap
          dist[v] = new_cost;      /// Update the distance
          pq.push({new_cost, v});  /// Push the vertex to the heap
        }
      }
    }
    return dist;
  }

  /// @brief Count the number of distinct walks of minimum cost between two
  /// vertices
  /// @param u
  /// @param target
  /// @param visited
  /// @param currentCost
  /// @param minCost
  /// @return
  int countPathsDijk(uint32_t u, uint32_t target, std::vector<bool>& visited,
                     int currentCost, const std::vector<int>& minCost) const {
    if (u == target)
      return currentCost == minCost[target]
                 ? 1
                 : 0;   /// If the current vertex is the target, return 1 if the
                        /// cost is the minimum cost, 0 otherwise
    visited[u] = true;  /// Mark the current vertex as visited
    int pathCount = 0;
    auto it_range = getOutboundEdges(u);
    for (auto it = it_range.first; it != it_range.second;
         ++it) {  /// Iterate over the outbound edges
      auto [v, weight] = *it;
      if (!visited[v] &&
          currentCost + getCost(u, v) <=
              minCost[v]) {  /// If the
                             /// vertex is not visited and the cost is less than
                             /// the minimum cost, increment the path count
        pathCount += countPathsDijk(v, target, visited,
                                    currentCost + getCost(u, v), minCost);
      }
    }
    visited[u] = false;
    return pathCount;
  }

  /// @brief Recursive function to count the number of distinct walks between
  /// two vertices in a DAG
  /// @param g The graph
  /// @param v  The current vertex
  /// @param t  The target vertex
  /// @param visited The vector of visited vertices
  /// @param pathCount The current path count
  void countPathsUtil(const Graph& g, uint32_t v, uint32_t t,
                      std::vector<bool>& visited, int& pathCount) {
    visited[v] = true;  /// Mark the current node as visited

    if (v == t) {
      pathCount++;  /// If current vertex is the target vertex, increment path
                    /// count
    } else {
      /// Go through all the vertices adjacent to this vertex
      auto edges = g.getOutEdges(v);
      for (auto& edge : edges) {
        if (!visited[edge.first]) {
          countPathsUtil(g, edge.first, t, visited, pathCount);
        }
      }
    }

    visited[v] =
        false;  /// Mark the current node as not visited to explore other paths
  }

  /// @brief Count the number of distinct walks between two vertices in a DAG
  /// @param g  The graph
  /// @param s  The source vertex
  /// @param t  The target vertex
  /// @return The number of distinct walks between the two vertices
  int countPaths(const Graph& g, uint32_t s, uint32_t t) {
    std::vector<bool> visited(g.getVertices(),
                              false);  /// Mark all vertices as not visited
    int pathCount = 0;                 /// Initialize path count as 0

    countPathsUtil(g, s, t, visited, pathCount);
    return pathCount;
  }

  /* --------------------------------------------- L4
   * -------------------------------- */

  void dfs(int v, std::unordered_map<int, bool>& visited,
           std::stack<int>& stack, std::unordered_map<int, bool>& recStack,
           bool& isCycle) {
    visited[v] = true;   /// current vertex is visited
    recStack[v] = true;  /// current vertex is in the recursion stack

    for (const auto& edge : outbound[v]) {
      if (!visited[edge.first]) {  /// we have not yet visited this vertex
        dfs(edge.first, visited, stack, recStack, isCycle);
        if (isCycle) return;
      } else if (recStack[edge.first]) {  /// we have visited this vertex and it
                                          /// is in the recursion stack
        isCycle = true;
        return;
      }
    }

    recStack[v] = false;
    stack.push(v);
  }

  bool isDAG(std::vector<int>& topologicalOrder) {
    std::unordered_map<int, bool> visited;   /// keep track of visited vertices
    std::unordered_map<int, bool> recStack;  /// keep track of vertices in the
                                             /// recursion stack
    std::stack<int> stack;  /// stack to store the topological order of vertices
    bool isCycle = false;

    for (const auto& vertex : outbound) {
      if (!visited[vertex.first]) {
        dfs(vertex.first, visited, stack, recStack, isCycle);
        if (isCycle) {
          return false;
        }
      }
    }

    topologicalOrder.clear();
    while (!stack.empty()) {  /// store the topological order of vertices
      topologicalOrder.push_back(stack.top());
      stack.pop();
    }

    return true;
  }

  std::vector<int> highestCostPath(int source, int target) {
    std::vector<int> topologicalOrder;
    if (!isDAG(topologicalOrder)) {
      std::cout << "The graph is not a DAG.\n";
      return {};
    }

    /// the graph is DAG

    std::vector<int> dist(outbound.size(), INT_MIN);  /// all dist is -inf
    std::vector<int> predecessor(outbound.size(),
                                 -1);  /// for path reconstruction
    dist[source] = 0;

    for (int u : topologicalOrder) {
      if (dist[u] != INT_MIN) {       /// if vertex u is reachable
        for (auto i : outbound[u]) {  /// for each outbound edge from u
          int v = i.first;
          int weight = getCost(u, v);
          if (dist[v] < dist[u] + weight) {  // if we can relax the edge
            dist[v] = dist[u] + weight;
            predecessor[v] = u;
          }
        }
      }
    }

    std::vector<int> path;
    for (int at = target; at != -1; at = predecessor[at]) {
      path.push_back(at);
      if (at == source) break;
    }

    if (path.back() != source) {
      std::cout << "No path exists from source to target.\n";
      return {};
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  /// Bonus 2
  int countDistinctPaths(int source, int target) {
    std::vector<int> topologicalOrder;
    if (!isDAG(topologicalOrder)) {
      std::cout << "The graph is not a DAG.\n";
      return 0;
    }

    std::vector<int> pathCount(outbound.size(), 0);
    pathCount[source] = 1;

    for (int u : topologicalOrder) {
      for (auto& edge : outbound[u]) {
        int v = edge.first;
        pathCount[v] += pathCount[u];
      }
    }

    return pathCount[target];
  }

  /// Bonus 3
  int countDistinctLowestCostPaths(int source, int target) {
    std::vector<int> topologicalOrder;
    if (!isDAG(topologicalOrder)) {
      std::cout << "The graph is not a DAG.\n";
      return 0;
    }

    std::vector<int> dist(outbound.size(),
                          INT_MAX);  /// we need low cost, so all are inf
    std::vector<int> count(outbound.size(), 0);  /// number of paths
    dist[source] = 0;
    count[source] = 1;

    for (int u : topologicalOrder) {
      if (dist[u] != INT_MAX) {
        for (const auto& edge : outbound[u]) {
          int v = edge.first;
          int weight = getCost(u, v);
          if (dist[u] + weight <
              dist[v]) {  /// if we can relax the edge, update the distance and
                          /// the number of paths
            dist[v] = dist[u] + weight;
            count[v] = count[u];
          } else if (dist[u] + weight ==
                     dist[v]) {  /// if we have another path, increment the
                                 /// number of paths
            count[v] += count[u];
          }
        }
      }
    }

    return count[target];
  }

  /* --------------------------------------------------------------- */
  /* L5 */

  std::vector<int> findHamiltonianCycle() {
    std::vector<int> cycle;
    std::vector<int> path;
    std::vector<bool> visited(vertices, false);
    cycle.push_back(0);
    visited[0] = true;
    auto result = hamiltonian_cycle(0, visited, cycle);
    if (result) {
      return cycle;
    }
    return {};
  }

  bool hamiltonian_cycle(int v, std::vector<bool>& visited,
                         std::vector<int>& cycle) {
    if (cycle.size() == vertices) {
      if (isEdge(v, 0)) {
        cycle.push_back(0);
        return true;
      }
      return false;
    }

    // Sort edges based on cost
    std::vector<std::pair<int, int>> sortedEdges;

    for (const auto& edge : outbound[v]) {
      sortedEdges.push_back({edge.first, getCost(v, edge.first)});
    }

    std::sort(sortedEdges.begin(), sortedEdges.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second < b.second;
              });

    for (const auto& edge : sortedEdges) {
      int u = edge.first;
      if (!visited[u]) {
        visited[u] = true;
        cycle.push_back(u);
        if (hamiltonian_cycle(u, visited, cycle)) {
          return true;
        }
        visited[u] = false;
        cycle.pop_back();
      }
    }

    return false;
  }

  /*-------------*/
  void branchAndBound(std::vector<int>& path, std::vector<bool>& visited,
                      int currentCost, int level) {
    if (level == vertices - 1) {
      if (isEdge(path[level], 0)) {
        int totalCost = currentCost + getCost(path[level], 0);
        if (totalCost < minCost) {
          minCost = totalCost;
          bestPath = path;
          bestPath.push_back(0);
        }
      }
      return;
    }

    for (const auto& edge : outbound[path[level]]) {
      int city = edge.first;
      int cost = getCost(path[level], city);
      if (!visited[city] && cost != -1) {
        int temp = currentCost + cost;
        if (temp < minCost) {
          visited[city] = true;
          path.push_back(city);
          branchAndBound(path, visited, temp, level + 1);
          visited[city] = false;
          path.pop_back();
        }
      }
    }
  }

  std::vector<int> findHamiltonianCycle2() {
    std::vector<int> path;
    std::vector<bool> visited(vertices, false);
    path.push_back(0);
    visited[0] = true;
    branchAndBound(path, visited, 0, 0);
    return bestPath;
  }
};

class UI {
 private:
  Graph graph;

 public:
  UI() {}

  ~UI() {}

  void displayMenu() {
    std::cout << "\n\nGraph Operations Menu:\n";
    std::cout << "1. Get the number of vertices\n";
    std::cout << "2. List all vertices\n";
    std::cout << "3. Check if an edge exists between two vertices and get "
                 "Edge ID\n";
    std::cout << "4. Get the in-degree and out-degree of a vertex\n";
    std::cout << "5. List all outbound edges of a vertex\n";
    std::cout << "6. List all inbound edges of a vertex\n";
    std::cout << "7. Get the endpoints of an edge by Edge ID\n";
    std::cout << "8. Modify the cost of an edge\n";
    std::cout << "9. Add a vertex\n";
    std::cout << "10. Remove a vertex\n";
    std::cout << "11. Add an edge\n";
    std::cout << "12. Remove an edge\n";
    std::cout << "13. Copy the graph\n";
    std::cout << "14. Read the graph from a file\n";
    std::cout << "15. Write the graph to a file\n";
    std::cout << "16. Create a random graph\n";
    // std::cout << "|==========================|\n";
    // std::cout << "17. Backwards BFS\n";
    // std::cout << "18. Strongly Connected Components\n";
    // std::cout << "|==========================|\n";
    // std::cout << "19. Find the lowest cost walk\n";
    // std::cout << "20. The number of distinct walks of minimum cost between
    // the "
    //              "given vertices.\n";
    // std::cout << "21. Find the number of distinct walks of between "
    //              "the given vertices.\n";
    // std::cout << "|==========================|\n";
    // std::cout << "22. Find the highest cost path between two vertices.\n";
    // std::cout << "23. Bonus 1\n";
    // std::cout << "24. Number of distinct paths between two vertices.\n";
    // std::cout << "25. Number of distinct lowest cost paths between two "
    //              "vertices.\n";
    std::cout << "|==========================|\n";
    std::cout << "26. Find the lowest cost Hamiltonian cycle.\n";
    std::cout << "|==========================|\n";
    std::cout << "0. Exit\n\n";
    std::cout << "Enter your choice: ";
  }

  void processInput(int choice) {
    uint32_t v1, v2, vertices, edges;
    int cost;
    std::string filename;
    switch (choice) {
      case 1:
        std::cout << "Number of vertices: " << graph.getVertices() << std::endl;
        break;
      case 2:
        std::cout << "Vertices list:\n";
        for (auto vertex : graph.getVerticesList()) {
          std::cout << vertex << std::endl;
        }
        break;
      case 3:
        std::cout << "Enter source and target vertices: ";
        std::cin >> v1 >> v2;
        if (graph.isEdge(v1, v2)) {
          std::cout << "Edge exists with ID: " << graph.getEdgeID(v1, v2)
                    << std::endl;
        } else {
          std::cout << "No edge exists.\n";
        }
        break;
      case 4:
        std::cout << "Enter vertex: ";
        std::cin >> v1;
        std::cout << "In-degree: " << graph.getInDegree(v1)
                  << ", Out-degree: " << graph.getOutDegree(v1) << std::endl;
        break;
      case 5:
        std::cout << "Enter vertex: ";
        std::cin >> v1;
        iterateOutEdges(v1);
        break;
      case 6:
        std::cout << "Enter vertex: ";
        std::cin >> v1;
        iterateInEdges(v1);
        break;
      case 7: {
        std::cout << "Enter Edge ID: ";
        std::cin >> edges;
        std::pair<uint32_t, uint32_t> endpoints = graph.getEndpoints(edges);
        if (endpoints.first != static_cast<uint32_t>(-1)) {
          std::cout << "Source: " << endpoints.first
                    << ", Target: " << endpoints.second << std::endl;
        } else {
          std::cout << "Edge ID does not exist.\n";
        }
        break;
      }
      case 8:
        std::cout << "Enter source, target vertices and new cost: ";
        std::cin >> v1 >> v2 >> cost;
        graph.setCost(v1, v2, cost);
        std::cout << "Cost updated.\n";
        break;
      case 9:
        std::cout << "Enter vertex to add: ";
        std::cin >> v1;
        graph.addVertex(v1);
        std::cout << "Vertex added.\n";
        break;
      case 10:
        std::cout << "Enter vertex to remove: ";
        std::cin >> v1;
        graph.removeVertex(v1);
        std::cout << "Vertex removed.\n";
        break;
      case 11:
        std::cout << "Enter source, target vertices and cost: ";
        std::cin >> v1 >> v2 >> cost;
        graph.addEdge(v1, v2, cost);
        std::cout << "Edge added.\n";
        break;
      case 12:
        std::cout << "Enter source and target vertices of the edge to remove:";
        std::cin >> v1 >> v2;
        graph.removeEdge(v1, v2);
        std::cout << " Edge removed.\n ";
        break;
      case 13: {
        Graph copy = graph.copyGraph();
        std::cout << "Graph copied. Copy has " << copy.getVertices()
                  << " vertices and " << copy.getEdges() << " edges.\n";
      } break;
      case 14:
        std::cout << "Enter filename to read graph from: ";
        std::cin >> filename;
        readGraphFromFile(filename);
        std::cout << "Graph loaded from file.\n";
        break;
      case 15:
        std::cout << "Enter filename to write graph to: ";
        std::cin >> filename;
        writeGraphToFile(filename);
        std::cout << "Graph written to file.\n";
        break;
      case 16:
        std::cout
            << "Enter number of vertices and edges for the random graph: ";
        std::cin >> vertices >> edges;
        graph = Graph::createRandomGraph(vertices, edges);
        std::cout << "Random graph created.\n";
        break;
      case 17:  /// Backwards BFS, L2 Assignment
        std::cout << "Enter starting vertex for backwards BFS: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for backwards BFS: ";
        std::cin >> v2;

        std::cout << "Backwards BFS: ";
        for (auto vertex : graph.findLowestLengthPath(v1, v2)) {
          std::cout << vertex << " ";
        }
        std::cout << std::endl;
        break;
      case 18:  /// Strongly Connected Components, L2 Bonus
        std::cout << "Strongly Connected Components:\n";
        for (const auto& component : graph.findStronglyConnectedComponents()) {
          for (const auto& vertex : component) {
            std::cout << vertex << " ";
          }
          std::cout << std::endl;
        }
        break;
      case 19:  /// Lowest Cost Walk, L3 Assignment
        std::cout << "Enter starting vertex for the lowest cost walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the lowest cost walk: ";
        std::cin >> v2;
        graph.findLowestCostWalk(v1, v2);
        break;
      case 20:  /// Number of Distinct Walks of Minimum Cost, L3 Bonus
      {
        std::cout << "Enter starting vertex for the walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the walk: ";
        std::cin >> v2;

        std::vector<int> minCost = graph.dijkstra(v1);
        std::vector<bool> visited(graph.getVertices(), false);

        int pathCount = graph.countPathsDijk(v1, v2, visited, 0, minCost);
        std::cout << "The number of distinct walks of minimum cost between "
                  << v1 << " and " << v2 << " is " << pathCount << ".\n";
        break;
      }
      case 21:
        std::cout << "Enter starting vertex for the walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the walk: ";
        std::cin >> v2;
        std::cout << "The number of distinct walks of between " << v1 << " and "
                  << v2 << " is " << graph.countPaths(graph, v1, v2) << ".\n";
        break;
      case 22:  /// Highest Cost Path, L4 Assignment
      {
        std::cout << "Enter starting vertex for the highest cost path: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the highest cost path: ";
        std::cin >> v2;
        std::vector<int> path = graph.highestCostPath(v1, v2);
        std::cout << "Highest cost path: ";
        for (int i = 0; i < path.size(); ++i) {
          std::cout << path[i];
          if (i < path.size() - 1) {
            std::cout << " -> ";
          }
        }
        std::cout << std::endl;
        break;
      }
      case 23:
        std::cout << "Bonus 1\n";
        break;
      case 24:  // Bonus 2
        std::cout << "Enter starting vertex for the walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the walk: ";
        std::cin >> v2;
        std::cout << "The number of distinct paths between " << v1 << " and "
                  << v2 << " is " << graph.countDistinctPaths(v1, v2) << ".\n";
        break;

      case 25:  // Bonus 3
        std::cout << "Enter starting vertex for the walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the walk: ";
        std::cin >> v2;
        std::cout << "The number of distinct lowest cost paths between " << v1
                  << " and " << v2 << " is "
                  << graph.countDistinctLowestCostPaths(v1, v2) << ".\n";
        break;
      case 26:  // L5 Assignment
      {
        std::vector<int> cycle = graph.findHamiltonianCycle();
        if (cycle.size() > 0) {
          std::cout << "Hamiltonian cycle found: ";
          for (int i = 0; i < cycle.size(); ++i) {
            std::cout << cycle[i];
            if (i < cycle.size() - 1) {
              std::cout << " -> ";
            }
          }
          std::cout << std::endl;
        } else {
          std::cout << "No Hamiltonian cycle found.\n";
        }
        break;
      }
      case 27: {
        std::vector<int> cycle = graph.findHamiltonianCycle2();
        if (cycle.size() > 0) {
          std::cout << "Hamiltonian cycle found: ";
          for (int i = 0; i < cycle.size(); ++i) {
            std::cout << cycle[i];
            if (i < cycle.size() - 1) {
              std::cout << " -> ";
            }
          }
          std::cout << std::endl;
        } else {
          std::cout << "No Hamiltonian cycle found.\n";
        }
        break;
      }
      case 0:
        std::cout << "Exiting...\n";
        break;
      default:
        std::cout << "Invalid choice, please try again.\n";
    }
  }

  void run() {
    int choice;
    do {
      displayMenu();
      std::cin >> choice;
      processInput(choice);
    } while (choice != 0);
  }

  void readGraphFromFile(std::string filename) {
    std::ifstream file(filename);
    if (file.is_open()) {
      graph = Graph();

      uint32_t vertices, edges;
      file >> vertices >> edges;

      graph.setVertices(vertices);
      graph.setEdges(edges);

      uint32_t source, target;
      int cost;
      for (uint32_t i = 0; i < edges; i++) {
        file >> source >> target >> cost;
        graph.addEdge(source, target, cost);
      }

      file.close();
    } else {
      std::cerr << "Could not open file: " << filename << std::endl;
    }
  };

  void printGraph() {
    std::cout << "Vertices: " << graph.getVertices() << std::endl;
    std::cout << "Edges: " << graph.getEdges() << std::endl;

    auto verticesList = graph.getVerticesList();
    for (auto vertex : verticesList) {
      std::cout << "Vertex: " << vertex
                << " - InDegree: " << graph.getInDegree(vertex)
                << ", OutDegree: " << graph.getOutDegree(vertex) << std::endl;

      for (auto edgeTarget : verticesList) {
        if (graph.isEdge(vertex, edgeTarget)) {
          std::cout << "  Edge to: " << edgeTarget
                    << ", Cost: " << graph.getCost(vertex, edgeTarget)
                    << std::endl;
        }
      }
    }
  }

  void writeGraphToFile(std::string filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
      file << graph.getVertices() << " " << graph.getEdges() << std::endl;

      auto verticesList = graph.getVerticesList();
      for (auto vertex : verticesList) {
        auto outEdges = graph.getOutEdges(vertex);
        for (auto edge : outEdges) {
          file << vertex << " " << edge.first << " "
               << graph.getCost(vertex, edge.first) << std::endl;
        }
      }

      file.close();
    } else {
      std::cerr << "Could not open file: " << filename << std::endl;
    }
  }

  void iterateOutEdges(uint32_t vertex) {
    auto outEdges = graph.getOutboundEdges(vertex);
    for (auto it = outEdges.first; it != outEdges.second; it++) {
      std::cout << "Edge to: " << it->first
                << ", Cost: " << graph.getCost(vertex, it->first) << std::endl;
    }
  }

  void iterateInEdges(uint32_t vertex) {
    auto inEdges = graph.getInboundEdges(vertex);
    for (auto it = inEdges.first; it != inEdges.second; it++) {
      std::cout << "Edge from: " << it->first
                << ", Cost: " << graph.getCost(it->first, vertex) << std::endl;
    }
  }
};

int main() {
  UI ui;
  ui.run();
  return 0;
}
