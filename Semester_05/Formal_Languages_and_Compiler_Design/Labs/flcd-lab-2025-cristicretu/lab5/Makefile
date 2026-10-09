CC = gcc
CFLAGS = -Wall -Wextra -std=c11
LEX = flex
LEXFLAGS = 

TARGET = scanner

LEX_SOURCE = scanner.l
FA_SOURCE = fa.c
SYMBOL_TABLE_SOURCE = symbol_table.c

FA_OBJ = fa.o
SYMBOL_TABLE_OBJ = symbol_table.o
LEX_OBJ = lex.yy.o

LEX_OUTPUT = lex.yy.c

.PHONY: all clean test test1 test2 test3

all: $(TARGET)

$(TARGET): $(LEX_OBJ) $(FA_OBJ) $(SYMBOL_TABLE_OBJ)
	$(CC) $(CFLAGS) -o $@ $^

$(LEX_OUTPUT): $(LEX_SOURCE)
	$(LEX) $(LEXFLAGS) $<

$(LEX_OBJ): $(LEX_OUTPUT)
	$(CC) $(CFLAGS) -c $< -o $@

$(FA_OBJ): $(FA_SOURCE) fa.h
	$(CC) $(CFLAGS) -c $< -o $@

$(SYMBOL_TABLE_OBJ): $(SYMBOL_TABLE_SOURCE) symbol_table.h
	$(CC) $(CFLAGS) -c $< -o $@

test1: $(TARGET)
	./$(TARGET) program1.txt

test2: $(TARGET)
	./$(TARGET) program2.txt

test3: $(TARGET)
	./$(TARGET) program3_error.txt

test: test1 test2 test3

clean:
	rm -f $(TARGET) $(LEX_OUTPUT) $(LEX_OBJ) $(FA_OBJ) $(SYMBOL_TABLE_OBJ)
	rm -rf *.dSYM

