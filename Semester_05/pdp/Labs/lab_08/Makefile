CXX = clang++
CXXFLAGS = -std=c++17 -O2 -Wall

dsm: main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

run: dsm
	./dsm 3

clean:
	rm -f dsm



