#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <list>

class Graph
{
private:
    struct Edge
    {
        int source, target, cost;
        Edge(int s, int t, int c) : source(s), target(t), cost(c) {}
    };

    int vertices = 0, edges = 0;
    std::unordered_map<int, std::list<Edge>> adj_out;
    std::unordered_map<int, std::list<Edge>> adj_in;
    std::unordered_map<int, Edge *> edge_map;

    int generateEdgeID()
    {
        return edges++;
    }

public:
    Graph() {}

    Graph(const Graph &) = delete;
    Graph &operator=(const Graph &) = delete;

    ~Graph()
    {
        for (auto &pair : edge_map)
            delete pair.second;
    }

    void addVertex(int vertex)
    {
        adj_out[vertex];
        adj_in[vertex];
    }

    void addEdge(int source, int target, int cost)
    {
        int edgeID = generateEdgeID();
        Edge *newEdge = new Edge(source, target, cost);

        adj_out[source].push_back(*newEdge);
        adj_in[target].push_back(*newEdge);
        edge_map[edgeID] = newEdge;
    }

    bool isEdge(int source, int target) const
    {
        auto it = adj_out.find(source);
        if (it != adj_out.end())
        {
            for (const auto &edge : it->second)
            {
                if (edge.target == target)
                    return true;
            }
        }
        return false;
    }

    int getEdgeCost(int source, int target) const
    {
        auto it = adj_out.find(source);
        if (it != adj_out.end())
        {
            for (const auto &edge : it->second)
            {
                if (edge.target == target)
                    return edge.cost;
            }
        }
        return -1; // Indicate not found/error
    }

    void setEdgeCost(int source, int target, int newCost)
    {
        auto it = adj_out.find(source);
        if (it != adj_out.end())
        {
            for (auto &edge : it->second)
            {
                if (edge.target == target)
                {
                    edge.cost = newCost;
                    edge_map[edge.source]->cost = newCost; // Update in edge_map for consistency
                    break;
                }
            }
        }
    }

    int getVerticesCount() const
    {
        return vertices;
    }

    int getEdgesCount() const
    {
        return edges;
    }

    size_t getOutDegree(int vertex) const
    {
        auto it = adj_out.find(vertex);
        return it != adj_out.end() ? it->second.size() : 0;
    }

    const std::list<Edge> getOutEdges(int vertex) const
    {
        static std::list<Edge> empty;
        auto it = adj_out.find(vertex);
        return it != adj_out.end() ? it->second : empty;
    }
};
class UI
{
private:
    Graph graph;

public:
    UI()
    {
    }

    ~UI()
    {
    }

    void readGraphFromFile(std::string filename)
    {
        std::ifstream file(filename);
        if (file.is_open())
        {
            int vertices, edges;
            file >> vertices >> edges;

            for (int i = 0; i < vertices; i++)
            {
            }
        }
    };

    void printGraph()
    {
        int vertices, edges;
        this->graph.get_vertices();
        this->graph.get_edges();

        for (auto edge : this->graph.get_out_edges())
        {
            std::cout << "Edge ID:" << std::get<2>(edge) << " " << std::get<0>(edge) << " -> " << std::get<1>(edge) << " with cost " << this->graph.get_cost().at(std::get<2>(edge)) << std::endl;
        }
    }
};

int main()
{
    UI ui;

    ui.readGraphFromFile("graph1k.txt");
    ui.printGraph();
    return 0;
}
