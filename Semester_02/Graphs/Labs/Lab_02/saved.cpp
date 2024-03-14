#include <iostream>
#include <fstream>
#include <vector>
#include <map>

class Graph
{
private:
    int vertices, edges;
    std::vector<std::tuple<int, int, int>> in_edge;
    std::vector<std::tuple<int, int, int>> out_edge;
    std::map<int, int> cost;

public:
    Graph(int vertices = 0, int edges = 0)
    {
        this->vertices = vertices;
        this->edges = edges;
    }

    ~Graph()
    {
        in_edge.clear();
        out_edge.clear();
        cost.clear();
    }

    void set_edges(int edges)
    {
        this->edges = edges;
    }

    void set_vertices(int vertices)
    {
        this->vertices = vertices;
    }

    int get_edges()
    {
        return this->edges;
    }

    int get_vertices()
    {
        return this->vertices;
    }

    bool is_edge(int source, int target)
    {
        for (auto &edge : out_edge)
        {
            if (std::get<0>(edge) == source && std::get<1>(edge) == target)
            {
                return true;
            }
        }
        return false;
    }

    int get_edge_id(int source, int target)
    {
        for (auto i : out_edge)
        {
            if (std::get<0>(i) == source && std::get<1>(i) == target)
            {
                return std::get<2>(i);
            }
        }
        return -1;
    }

    void set_edge_cost(int source, int target, int cost)
    {
        if (this->is_edge(source, target))
        {
            int edge_id = this->get_edge_id(source, target);
            this->cost[edge_id] = cost;
        }
    }

    void get_edge_cost(int source, int target, int &cost)
    {
        if (this->is_edge(source, target))
        {
            int edge_id = this->get_edge_id(source, target);
            cost = this->cost[edge_id];
        }
    }

    void add_edge(int source, int target, int cost)
    {
        if (this->is_edge(source, target))
        {
            return;
        }

        int edge_id = this->edges;
        this->edges++;
        this->out_edge.push_back(std::make_tuple(source, target, edge_id));
        this->in_edge.push_back(std::make_tuple(target, source, edge_id));
        this->cost[edge_id] = cost;
    }

    const std::vector<std::tuple<int, int, int>> &get_out_edges() const
    {
        return out_edge;
    }

    const std::vector<std::tuple<int, int, int>> &get_in_edges() const
    {
        return in_edge;
    }

    const std::map<int, int> &get_cost() const
    {
        return cost;
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

            this->graph.set_vertices(vertices);
            this->graph.set_edges(edges);

            for (int i = 0; i < edges; i++)
            {
                int source, target, cost;
                file >> source >> target >> cost;
                this->graph.add_edge(source, target, cost);
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
