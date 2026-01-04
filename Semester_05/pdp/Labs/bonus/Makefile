CXX = clang++
CXXFLAGS = -std=c++17 -O3 -framework OpenCL

all: poly_gpu

poly_gpu: main.cpp
	$(CXX) $(CXXFLAGS) main.cpp -o poly_gpu

run: poly_gpu
	./poly_gpu

clean:
	rm -f poly_gpu



