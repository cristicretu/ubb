#include <iostream>
#include <fstream>
#include <vector>

// Design and implement an abstract data type directed graph and a function (either a member function or an external one, as your choice) for reading a directed graph from a text file.
/*
The vertices will be specified as integers from 0 to n-1, where n is the number of vertices.

Edges may be specified either by the two endpoints (that is, by the source and target), or by some abstract data type Edge_id (that data type may be a pointer or reference to the edge representation, but without exposing the implementation details of the graph).

Additionally, create a map that associates to an edge an integer value (for instance, a cost).*/

class DirectedGraph
{
public:
    DirectedGraph(int n) : n(n)
    {
        adj = new int *[n];
        for (int i = 0; i < n; i++)
        {
            adj[i] = new int[n];
            for (int j = 0; j < n; j++)
            {
                adj[i][j] = 0;
            }
        }
    }

    void addEdge(int source, int target, int cost)
    {
        adj[source][target] = cost;
    }

    void printGraph()
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                std::cout << adj[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

private:
    int n;
    int **adj;
};

std::pair<int, std::vector<std::tuple<int, int, int>>> readGraphFromFile(const char *filename)
{
    std::ifstream fin(filename);

    std::vector<std::tuple<int, int, int>> edges;

    if (!fin)
    {
        std::cerr << "Error opening file" << std::endl;
        return std::make_pair(0, edges);
    }

    int edges_len, vertices;
    fin >> vertices >> edges_len;
    for (int i = 0; i < edges_len; i++)
    {
        int source, target, cost;
        fin >> source >> target >> cost;

        // add edge to graph
        edges.push_back(std::make_tuple(source, target, cost));
    }

    return std::make_pair(vertices, edges);
}

int main()
{
    int vertices;
    std::vector<std::tuple<int, int, int>> edges;

    std::tie(vertices, edges) = readGraphFromFile("graph.txt");

    DirectedGraph graph(vertices);

    for (auto edge : edges)
    {
        int source, target, cost;
        std::tie(source, target, cost) = edge;
        graph.addEdge(source, target, cost);
    }

    graph.printGraph();
    return 0;
}
