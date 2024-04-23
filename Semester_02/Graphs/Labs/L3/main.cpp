#include <climits>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

class Graph {
 private:
  uint32_t vertices = 0, edges = 0;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> outbound;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> inbound;
  std::unordered_map<int, int> cost;

  /// For the biconnected components
  std::vector<int> discoveryLevel, lowReach;
  std::stack<uint32_t> stack;
  std::vector<std::vector<uint32_t>> articulated_components;

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
      this->inbound[target].push_back({source, edgeID});
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

  // void findLowestCostWalk(uint32_t start, uint32_t end) {
  //   // Initialize the distance matrix
  //   std::vector<std::vector<int>> dist(vertices,
  //                                      std::vector<int>(vertices, INT_MAX));
  //   for (uint32_t i = 0; i < vertices; i++) {
  //     dist[i][i] = 0;
  //   }

  //   for (auto& source : outbound) {
  //     for (auto& edge : source.second) {
  //       uint32_t target = edge.first;
  //       int edgeCost = cost[edge.second];
  //       dist[source.first][target] = edgeCost;
  //     }
  //   }

  //   // Matrix multiplication to find all-pairs shortest paths
  //   for (uint32_t step = 1; step < vertices; step *= 2) {
  //     std::vector<std::vector<int>> newDist(
  //         vertices, std::vector<int>(vertices, INT_MAX));
  //     for (uint32_t i = 0; i < vertices; i++) {
  //       for (uint32_t j = 0; j < vertices; j++) {
  //         for (uint32_t k = 0; k < vertices; k++) {
  //           if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
  //             int possibleDist = dist[i][k] + dist[k][j];
  //             if (possibleDist < newDist[i][j]) {
  //               newDist[i][j] = possibleDist;
  //             }
  //           }
  //         }
  //       }
  //     }
  //     dist = newDist;
  //   }

  //   // Check for negative cycles
  //   for (uint32_t i = 0; i < vertices; i++) {
  //     if (dist[i][i] < 0) {
  //       std::cout << "Graph contains a negative cost cycle\n";
  //       return;
  //     }
  //   }

  //   // Print the result for the specific path
  //   if (dist[start][end] == INT_MAX) {
  //     std::cout << "No path exists from vertex " << start << " to vertex "
  //               << end << "\n";
  //   } else {
  //     std::cout << "The lowest cost from vertex " << start << " to vertex "
  //               << end << " is " << dist[start][end] << "\n";
  //   }
  // }

  // Function to multiply two matrices
  std::vector<std::vector<int>> multiplyMatrix(
      const std::vector<std::vector<int>>& A,
      const std::vector<std::vector<int>>& B, int V) {
    std::vector<std::vector<int>> C(V, std::vector<int>(V, INT_MAX));
    for (int i = 0; i < V; ++i) {
      for (int j = 0; j < V; ++j) {
        for (int k = 0; k < V; ++k) {
          if (A[i][k] != INT_MAX && B[k][j] != INT_MAX) {
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

  // Function to check for negative cycles using matrix multiplication
  bool detectNegativeCycle(const std::vector<std::vector<int>>& W, int V) {
    std::vector<std::vector<int>> M = W;  // Start with the base matrix
    for (int m = 1; m < V; ++m) {
      M = multiplyMatrix(M, W, V);
      for (int i = 0; i < V; ++i) {
        if (M[i][i] < 0) {
          return true;  // Negative cycle detected
        }
      }
    }
    return false;
  }

  // Main function to find the lowest cost walk using matrix multiplication
  void findLowestCostWalk(Graph& graph, uint32_t source, uint32_t target) {
    uint32_t V = graph.getVertices();
    std::vector<std::vector<int>> W(V, std::vector<int>(V, INT_MAX));

    // Initialize the matrix W
    for (uint32_t i = 0; i < V; ++i) {
      W[i][i] = 0;
      for (const auto& edge : graph.getOutEdges(i)) {
        W[i][edge.first] = graph.getCost(i, edge.first);
      }
    }

    // Check for negative cycle
    if (detectNegativeCycle(W, V)) {
      std::cout << "The graph contains a negative cost cycle.\n";
      return;
    }

    // Compute the shortest paths
    std::vector<std::vector<int>> D = W;
    for (int m = 1; m < V; ++m) {
      D = multiplyMatrix(D, W, V);
    }

    // Output the lowest cost from source to target
    if (D[source][target] == INT_MAX) {
      std::cout << "No path exists from " << source << " to " << target
                << ".\n";
    } else {
      std::cout << "The lowest cost from " << source << " to " << target
                << " is " << D[source][target] << ".\n";
    }
  }

  void floydWarshall(Graph& graph) {
    int V = graph.getVertices();
    std::vector<std::vector<int>> dist(V, std::vector<int>(V, INT_MAX));
    std::vector<std::vector<int>> count(V, std::vector<int>(V, 0));

    // Initialize the distance and path count matrices
    for (int i = 0; i < V; ++i) {
      for (const auto& edge : graph.getOutEdges(i)) {
        dist[i][edge.first] = graph.getCost(i, edge.first);
        count[i][edge.first] = 1;
      }
      dist[i][i] = 0;
      count[i][i] =
          1;  // There is one way to stay at the same node (do nothing)
    }

    // Floyd-Warshall algorithm
    for (int k = 0; k < V; ++k) {
      for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
          if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
            int newDist = dist[i][k] + dist[k][j];
            if (newDist < dist[i][j]) {
              dist[i][j] = newDist;
              count[i][j] = count[i][k] * count[k][j];
            } else if (newDist == dist[i][j]) {
              count[i][j] += count[i][k] * count[k][j];
            }
          }
        }
      }
    }

    // Output the number of minimum cost paths between all pairs
    for (int i = 0; i < V; ++i) {
      for (int j = 0; j < V; ++j) {
        std::cout << "The number of minimum-cost paths from " << i << " to "
                  << j << " is " << count[i][j] << "\n";
      }
    }
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
    std::cout
        << "3. Check if an edge exists between two vertices and get Edge ID\n";
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
    std::cout << "|==========================|\n";
    std::cout << "17. Backwards BFS\n";
    std::cout << "18. Strongly Connected Components\n";
    std::cout << "|==========================|\n";
    std::cout << "19. Find the lowest cost walk\n";
    std::cout << "20. The number of distinct walks of minimum cost between the "
                 "given vertices.\n";
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
        graph.findLowestCostWalk(graph, v1, v2);
        break;
      case 20:  /// Number of Distinct Walks of Minimum Cost, L3 Bonus
        std::cout << "Enter starting vertex for the walk: ";
        std::cin >> v1;
        std::cout << "Enter ending vertex for the walk: ";
        std::cin >> v2;
        graph.floydWarshall(graph);
        break;
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
