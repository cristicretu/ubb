CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -pedantic -O2
TARGET = fa_processor

.PHONY: all clean test

all: $(TARGET)

$(TARGET): fa_processor.c
	$(CC) $(CFLAGS) -o $(TARGET) fa_processor.c

test: $(TARGET)
	@echo "Testing identifier FA..."
	./$(TARGET) identifier print
	@echo "\nTesting number FA..."
	./$(TARGET) number print
	@echo "\nTesting string FA..."
	./$(TARGET) string print
	@echo "\nValidating identifiers..."
	./$(TARGET) identifier validate myVar
	./$(TARGET) identifier validate _private
	./$(TARGET) identifier validate 123invalid || true
	@echo "\nValidating numbers..."
	./$(TARGET) number validate 42
	./$(TARGET) number validate 3.14
	./$(TARGET) number validate .5 || true
	@echo "\nValidating strings..."
	./$(TARGET) string validate '"hello"'
	./$(TARGET) string validate '""'
	./$(TARGET) string validate '"unclosed' || true

clean:
	rm -f $(TARGET)
