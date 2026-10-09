CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O3 -march=native -flto
LDFLAGS = -flto

TARGET = ll1parser
SRCS = main.c grammar.c ll1.c tree.c parser.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean run test bench

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET) grammar.txt ../lab6/output/pif.txt output.txt
	@echo ""
	@cat output.txt

test: $(TARGET)
	@echo "=== Running parser on lab6 PIF ==="
	./$(TARGET) grammar.txt ../lab6/output/pif.txt output.txt

# Benchmark: run 100 times and measure
bench: $(TARGET)
	@echo "=== Benchmarking parser (100 iterations) ==="
	@time for i in $$(seq 1 100); do ./$(TARGET) grammar.txt ../lab6/output/pif.txt output.txt > /dev/null 2>&1; done

clean:
	rm -f $(TARGET) $(OBJS) output.txt

# Dependencies
grammar.o: grammar.h
ll1.o: ll1.h grammar.h
tree.o: tree.h grammar.h
parser.o: parser.h grammar.h ll1.h tree.h
main.o: grammar.h ll1.h tree.h parser.h
