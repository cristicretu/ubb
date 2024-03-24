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

  std::vector<std::pair<uint32_t, uint32_t>> getOutEdges(
      uint32_t vertex) const {
    if (outbound.find(vertex) != outbound.end()) {
      return std::vector<std::pair<uint32_t, uint32_t>>(
          outbound.at(vertex).begin(), outbound.at(vertex).end());
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
};
class UI {
 private:
  Graph graph;

 public:
  UI() {}

  ~UI() {}

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
};

int main() {
  UI ui;

  ui.readGraphFromFile("graph1k.txt");
  ui.printGraph();
  return 0;
}
