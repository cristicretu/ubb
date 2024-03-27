#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

class Graph {
 private:
  uint32_t vertices = 0, edges = 0;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> outbound;
  std::unordered_map<int, std::list<std::pair<uint32_t, uint32_t>>> inbound;
  std::unordered_map<int, int> cost;

  int generateEdgeID() { return edges++; }

 public:
  Graph(uint32_t vertices = 0, uint32_t edges = 0)
      : vertices(vertices), edges(edges) {}

  ~Graph() {
    for (auto &edge : outbound) {
      edge.second.clear();
    }
    for (auto &edge : inbound) {
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
      for (const auto &edge : outbound.at(source)) {
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
          [edgeID](const std::pair<uint32_t, uint32_t> &edge) {
            return edge.second == edgeID;
          });
      inbound[target].remove_if(
          [edgeID](const std::pair<uint32_t, uint32_t> &edge) {
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
      for (auto &edge : std::vector<std::pair<uint32_t, uint32_t>>(
               outbound[vertex].begin(), outbound[vertex].end())) {
        removeEdge(vertex, edge.first);
      }

      for (auto &edge : std::vector<std::pair<uint32_t, uint32_t>>(
               inbound[vertex].begin(), inbound[vertex].end())) {
        removeEdge(edge.first, vertex);
      }

      outbound.erase(vertex);
      inbound.erase(vertex);

      vertices--;
    }
  }

  std::pair<uint32_t, uint32_t> getEndpoints(uint32_t edgeID) const {
    for (auto &edge : outbound) {
      for (auto &target : edge.second) {
        if (target.second == edgeID) {
          return {edge.first, target.first};
        }
      }
    }
    return {-1, -1};
  }

  Graph copyGraph() const {
    Graph copy(vertices, edges);
    for (auto &edge : outbound) {
      for (auto &target : edge.second) {
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
