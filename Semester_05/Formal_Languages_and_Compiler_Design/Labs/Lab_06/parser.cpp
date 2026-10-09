#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "antlr4-runtime.h"
#include "syntaxBaseListener.h"
#include "syntaxLexer.h"
#include "syntaxParser.h"

using namespace antlr4;
using namespace std;

class ProductionTable : public syntaxBaseListener {
  vector<string> productions;
  syntaxParser *parser;

public:
  ProductionTable(syntaxParser *p) : parser(p) {}

  void enterEveryRule(ParserRuleContext *ctx) override {
    productions.push_back(parser->getRuleNames()[ctx->getRuleIndex()]);
  }

  string getProductionString() {
    string result;
    for (size_t i = 0; i < productions.size(); i++) {
      if (i > 0)
        result += " -> ";
      result += productions[i];
      result += "\n";
    }
    return result;
  }
};

int main(int argc, const char *argv[]) {
  string inputFile = argv[1];

  cout << "Lexing " << inputFile << "..." << endl;
  if (system(("./lexer " + inputFile).c_str()) != 0) {
    cerr << "Lexer failed\n";
    return 1;
  }

  ifstream stream(inputFile);
  ANTLRInputStream input(stream);
  syntaxLexer lexer(&input);
  CommonTokenStream tokens(&lexer);
  syntaxParser parser(&tokens);

  ProductionTable tbl(&parser);
  parser.addParseListener(&tbl);

  auto tree = parser.program();

  ofstream out("output/parse_output.txt");
  out << "Productions:\n" << tbl.getProductionString() << "\n\n";
  out << "Parse tree:\n" << tree->toStringTree(&parser) << "\n";

  return 0;
}
