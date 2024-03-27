class Graph:
    def __init__(self, vertices=0, edges=0):
        self.vertices = vertices
        self.edges = edges
        self.outbound = {}
        self.inbound = {}
        self.cost = {}

    def generate_edge_id(self):
        edge_id = self.edges
        self.edges += 1
        return edge_id

    def set_vertices(self, vertices):
        self.vertices = vertices

    def set_edges(self, edges):
        self.edges = edges

    def set_cost(self, source, target, cost):
        if self.is_edge(source, target):
            self.cost[self.get_edge_id(source, target)] = cost

    def get_vertices(self):
        return self.vertices

    def get_edges(self):
        return self.edges

    def get_cost(self, source, target):
        if self.is_edge(source, target):
            edge_id = self.get_edge_id(source, target)
            return self.cost.get(edge_id, -1)
        return -1

    def get_outbound_edges(self, vertex):
        return self.outbound.get(vertex, [])

    def get_inbound_edges(self, vertex):
        return self.inbound.get(vertex, [])

    def get_vertices_list(self):
        return list(self.outbound.keys())

    def get_edge_id(self, source, target):
        for dest, edge_id in self.outbound.get(source, []):
            if dest == target:
                return edge_id
        return -1

    def is_edge(self, source, target):
        return self.get_edge_id(source, target) != -1

    def get_in_degree(self, vertex):
        return len(self.inbound.get(vertex, []))

    def get_out_degree(self, vertex):
        return len(self.outbound.get(vertex, []))

    def add_edge(self, source, target, cost):
        if not self.is_edge(source, target):
            edge_id = self.generate_edge_id()
            self.outbound.setdefault(source, []).append((target, edge_id))
            self.inbound.setdefault(target, []).append((source, edge_id))
            self.cost[edge_id] = cost

    def remove_edge(self, source, target):
        edge_id = self.get_edge_id(source, target)
        if edge_id != -1:
            self.outbound[source] = [pair for pair in self.outbound[source] if pair[1] != edge_id]
            self.inbound[target] = [pair for pair in self.inbound[target] if pair[1] != edge_id]
            del self.cost[edge_id]

    def add_vertex(self, vertex):
        if vertex not in self.outbound:
            self.outbound[vertex] = []
            self.inbound[vertex] = []
            self.vertices += 1

    def remove_vertex(self, vertex):
        if vertex in self.outbound:
            for _, edge_id in self.outbound[vertex]:
                source, target = self.get_endpoints(edge_id)
                self.remove_edge(source, target)
            for _, edge_id in self.inbound[vertex]:
                source, target = self.get_endpoints(edge_id)
                self.remove_edge(source, target)

            del self.outbound[vertex]
            del self.inbound[vertex]
            self.vertices -= 1

    def get_endpoints(self, edge_id):
        for source, edges in self.outbound.items():
            for target, eid in edges:
                if eid == edge_id:
                    return source, target
        return -1, -1

    def copy_graph(self):
        new_graph = Graph(self.vertices, self.edges)
        new_graph.outbound = {k: v[:] for k, v in self.outbound.items()}
        new_graph.inbound = {k: v[:] for k, v in self.inbound.items()}
        new_graph.cost = self.cost.copy()
        return new_graph

    @staticmethod
    def create_random_graph(vertices, max_edges):
        from random import randrange
        random_graph = Graph(vertices)
        while random_graph.get_edges() < max_edges:
            source = randrange(vertices)
            target = randrange(vertices)
            if source != target and not random_graph.is_edge(source, target):
                cost = randrange(100)
                random_graph.add_edge(source, target, cost)
        return random_graph
 

class UI:
    def __init__(self):
        self.graph = Graph()

    def display_menu(self):
        print("\nGraph Operations Menu:")
        print("1. Get the number of vertices")
        print("2. List all vertices")
        print("3. Check if an edge exists between two vertices and get Edge ID")
        print("4. Get the in-degree and out-degree of a vertex")
        print("5. List all outbound edges of a vertex")
        print("6. List all inbound edges of a vertex")
        print("7. Get the endpoints of an edge by Edge ID")
        print("8. Modify the cost of an edge")
        print("9. Add a vertex")
        print("10. Remove a vertex")
        print("11. Add an edge")
        print("12. Remove an edge")
        print("13. Copy the graph")
        print("14. Read the graph from a file")
        print("15. Write the graph to a file")
        print("16. Create a random graph")
        print("0. Exit")
        print("Enter your choice: ", end='')

    def process_input(self, choice):
        if choice == 1:
            print(f"Number of vertices: {self.graph.get_vertices()}")
        elif choice == 2:
            print("Vertices list:")
            for vertex in self.graph.get_vertices_list():
                print(vertex)
        elif choice == 3:
            source = int(input("Enter source vertex: "))
            target = int(input("Enter target vertex: "))
            if self.graph.is_edge(source, target):
                print(f"Edge exists with ID: {self.graph.get_edge_id(source, target)}")
            else:
                print("No edge exists.")
        elif choice == 4:
            vertex = int(input("Enter vertex: "))
            print(f"In-degree: {self.graph.get_in_degree(vertex)}")
            print(f"Out-degree: {self.graph.get_out_degree(vertex)}")
        elif choice == 5:
            vertex = int(input("Enter vertex: "))
            print("Outbound edges:")
            for target, edge_id in self.graph.get_outbound_edges(vertex):
                print(f"Target: {target}, Edge ID: {edge_id}")
        elif choice == 6:
            vertex = int(input("Enter vertex: "))
            print("Inbound edges:")
            for source, edge_id in self.graph.get_inbound_edges(vertex):
                print(f"Source: {source}, Edge ID: {edge_id}")

        elif choice == 7:
            edge_id = int(input("Enter Edge ID: "))
            source, target = self.graph.get_endpoints(edge_id)
            if source != -1:
                print(f"Source: {source}, Target: {target}")
            else:
                print("Edge not found.")

        elif choice == 8:
            source = int(input("Enter source vertex: "))
            target = int(input("Enter target vertex: "))
            cost = int(input("Enter new cost: "))
            self.graph.set_cost(source, target, cost)

        elif choice == 9:
            vertex = int(input("Enter vertex: "))
            self.graph.add_vertex(vertex)

        elif choice == 10:
            vertex = int(input("Enter vertex: "))
            self.graph.remove_vertex(vertex)

        elif choice == 11:
            source = int(input("Enter source vertex: "))
            target = int(input("Enter target vertex: "))
            cost = int(input("Enter cost: "))
            self.graph.add_edge(source, target, cost)

        elif choice == 12:
            source = int(input("Enter source vertex: "))
            target = int(input("Enter target vertex: "))
            self.graph.remove_edge(source, target)

        elif choice == 13:
            new_graph = self.graph.copy_graph()
            print("Graph copied successfully.")
            print("Original graph:")
            for vertex in self.graph.get_vertices_list():
                for target, edge_id in self.graph.get_outbound_edges(vertex):
                    print(f"{vertex} -> {target} ({self.graph.get_cost(vertex, target)})")
            print("Copied graph:")
            for vertex in new_graph.get_vertices_list():
                for target, edge_id in new_graph.get_outbound_edges(vertex):
                    print(f"{vertex} -> {target} ({new_graph.get_cost(vertex, target)})")


        elif choice == 14:
            filename = input("Enter filename to read graph from: ")
            self.read_graph_from_file(filename)
        elif choice == 15:
            filename = input("Enter filename to write graph to: ")
            self.write_graph_to_file(filename)

    def read_graph_from_file(self, filename):
        with open(filename, 'r') as file:
            vertices, edges = map(int, file.readline().split())
            self.graph.set_vertices(vertices)
            self.graph.set_edges(edges)
            for line in file:
                source, target, cost = map(int, line.split())
                self.graph.add_edge(source, target, cost)

    def write_graph_to_file(self, filename):
        with open(filename, 'w') as file:
            file.write(f"{self.graph.get_vertices()} {self.graph.get_edges()}\n")
            for vertex in self.graph.get_vertices_list():
                for target, edge_id in self.graph.get_outbound_edges(vertex):
                    file.write(f"{vertex} {target} {self.graph.get_cost(vertex, target)}\n")

    def run(self):
        while True:
            self.display_menu()
            choice = int(input())
            if choice == 0:
                break
            self.process_input(choice)

if __name__ == "__main__":
    ui = UI()
    ui.run()