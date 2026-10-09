.PHONY: all clean run test

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g
LEX = flex

all: scanner

scanner: scanner.l symbol_table.o
	$(LEX) scanner.l
	$(CC) $(CFLAGS) lex.yy.c symbol_table.o -o scanner

symbol_table.o: symbol_table.c symbol_table.h
	$(CC) $(CFLAGS) -c symbol_table.c -o symbol_table.o

test: scanner
	./scanner program1.txt
	./scanner program2.txt
	./scanner program3_error.txt || true

clean:
	rm -f scanner lex.yy.c symbol_table.o

run: scanner
	./scanner

