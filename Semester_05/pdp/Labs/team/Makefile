CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -pthread

TARGET = graph_coloring

all: $(TARGET)

$(TARGET): main.cpp graph.hpp coloring.hpp parallel_coloring.hpp
	$(CXX) $(CXXFLAGS) -o $@ main.cpp

debug: CXXFLAGS = -std=c++17 -Wall -Wextra -g -pthread -fsanitize=thread
debug: clean $(TARGET)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run debug
