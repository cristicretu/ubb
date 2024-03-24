class DirectedGraph:
    def __init__(self):
        self.__vertices = 0
        self.__edges = 0
        self.__inbound = {}
        self.__outbound = {}
        self.__cost = {}

    def get_vertices(self):
        return self.__vertices
    
    def get_edges(self):
        return self.__edges
    
    def get_inbound(self):
        return self.__inbound
    
    def get_outbound(self):
        return self.__outbound
    
    def get_cost(self):
        return self.__cost
    
    def set_number_of_vertices(self, vertices):
        self.__vertices = vertices

    def set_number_of_edges(self, edges):
        self.__edges = edges

    def add_vertex(self, vertex):
        if vertex not in self.__inbound:
            self.__inbound[vertex] = {}
            self.__outbound[vertex] = {}

    def add_edge(self, start, end, cost):
        self.add_vertex(start)
        self.add_vertex(end)

        self.__inbound[end][start] = self.__edges
        self.__outbound[start][end] = self.__edges
        self.__cost[self.__outbound[start][end]] = cost

        self.__edges += 1

    def remove_vertex(self, vertex):
        if vertex in self.__inbound:
            for start in self.__inbound[vertex]:
                del self.__outbound[start][vertex]
                del self.__cost[self.__inbound[vertex][start]]
            del self.__inbound[vertex]

        if vertex in self.__outbound:
            for end in self.__outbound[vertex]:
                del self.__inbound[end][vertex]
            del self.__outbound[vertex]


    def is_edge(self, start, end):
        return end in self.__outbound[start]
    
    