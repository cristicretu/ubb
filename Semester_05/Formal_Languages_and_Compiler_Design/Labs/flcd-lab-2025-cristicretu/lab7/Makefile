CC = gcc
CFLAGS = -Wall -Wextra -std=c99

TARGET = ll1parser
SRCS = main.c grammar.c ll1.c tree.c parser.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean run test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	./$(TARGET) grammar.txt ../lab6/output/pif.txt output.txt
	@echo ""
	@cat output.txt

test: $(TARGET)
	@echo "=== Running parser on lab6 PIF ==="
	./$(TARGET) grammar.txt ../lab6/output/pif.txt output.txt

clean:
	rm -f $(TARGET) $(OBJS) output.txt

grammar.o: grammar.h
ll1.o: ll1.h grammar.h
tree.o: tree.h grammar.h
parser.o: parser.h grammar.h ll1.h tree.h
main.o: grammar.h ll1.h tree.h parser.h

