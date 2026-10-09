TARGET = poly_multiply

all: $(TARGET)

$(TARGET): main.mm
	clang++ -std=c++17 -O3 -framework Metal -framework Foundation -o $@ main.mm

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
