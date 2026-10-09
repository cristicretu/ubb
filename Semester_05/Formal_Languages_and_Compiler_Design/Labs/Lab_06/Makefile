CC = gcc
CXX = g++
FLEX = flex
ANTLR = antlr

CFLAGS = -Wall
CXXFLAGS = -std=c++17 -I. -I/usr/local/include/antlr4-runtime -I/opt/homebrew/include/antlr4-runtime
LDFLAGS = -L/usr/local/lib -L/opt/homebrew/lib -lantlr4-runtime

LEXER = lexer
PARSER = parser

ANTLR_GEN = syntaxLexer.cpp syntaxParser.cpp syntaxBaseListener.cpp syntaxListener.cpp \
            syntaxLexer.h syntaxParser.h syntaxBaseListener.h syntaxListener.h \
            syntax.interp syntax.tokens syntaxLexer.interp syntaxLexer.tokens

.PHONY: all clean run run-all

all: $(LEXER) $(PARSER)

$(LEXER): lex.l fa.c fa.h
	$(FLEX) -o lex.yy.c lex.l
	$(CC) $(CFLAGS) -o $(LEXER) lex.yy.c fa.c

antlr-gen: syntax.g4
	$(ANTLR) -Dlanguage=Cpp -no-visitor -listener syntax.g4

$(PARSER): antlr-gen parser.cpp
	$(CXX) $(CXXFLAGS) -c syntaxLexer.cpp -o syntaxLexer.o
	$(CXX) $(CXXFLAGS) -c syntaxParser.cpp -o syntaxParser.o
	$(CXX) $(CXXFLAGS) -c syntaxBaseListener.cpp -o syntaxBaseListener.o
	$(CXX) $(CXXFLAGS) -c syntaxListener.cpp -o syntaxListener.o
	$(CXX) $(CXXFLAGS) -c parser.cpp -o parser.o
	$(CXX) $(CXXFLAGS) -o $(PARSER) syntaxLexer.o syntaxParser.o syntaxBaseListener.o syntaxListener.o parser.o $(LDFLAGS)

run: all
	./$(PARSER) program2.txt
	@echo ""
	@echo "=== PIF Output ==="
	@cat output/pif.txt
	@echo ""
	@echo "=== Parse Output ==="
	@cat output/parse_output.txt

run-all: all
	@echo "=== Testing program1.txt ==="
	./$(PARSER) program1.txt
	@echo ""
	@echo "=== Testing program2.txt ==="
	./$(PARSER) program2.txt

clean:
	rm -f $(LEXER) $(PARSER) lex.yy.c *.o $(ANTLR_GEN)
	rm -rf output
