
// Generated from syntax.g4 by ANTLR 4.13.2

#pragma once


#include "antlr4-runtime.h"




class  syntaxLexer : public antlr4::Lexer {
public:
  enum {
    PROGRAM = 1, END = 2, LET = 3, READ = 4, SAY = 5, IF = 6, ELSE = 7, 
    FOR = 8, FN = 9, RETURN = 10, DOT = 11, FLOORDIV = 12, GCD = 13, LCM = 14, 
    POWER = 15, EQ = 16, NEQ = 17, GTE = 18, LTE = 19, RELU = 20, PLUS = 21, 
    MINUS = 22, MULT = 23, DIV = 24, MOD = 25, GT = 26, LT = 27, AND = 28, 
    OR = 29, XOR = 30, FACTORIAL = 31, SIGMOID = 32, ASSIGN = 33, LPAREN = 34, 
    RPAREN = 35, LBRACE = 36, RBRACE = 37, LBRACKET = 38, RBRACKET = 39, 
    SEMI = 40, COMMA = 41, NUMBER = 42, STRING = 43, IDENTIFIER = 44, COMMENT = 45, 
    WS = 46
  };

  explicit syntaxLexer(antlr4::CharStream *input);

  ~syntaxLexer() override;


  std::string getGrammarFileName() const override;

  const std::vector<std::string>& getRuleNames() const override;

  const std::vector<std::string>& getChannelNames() const override;

  const std::vector<std::string>& getModeNames() const override;

  const antlr4::dfa::Vocabulary& getVocabulary() const override;

  antlr4::atn::SerializedATNView getSerializedATN() const override;

  const antlr4::atn::ATN& getATN() const override;

  // By default the static state used to implement the lexer is lazily initialized during the first
  // call to the constructor. You can call this function if you wish to initialize the static state
  // ahead of time.
  static void initialize();

private:

  // Individual action functions triggered by action() above.

  // Individual semantic predicate functions triggered by sempred() above.

};

