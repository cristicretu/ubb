
// Generated from syntax.g4 by ANTLR 4.13.2


#include "syntaxListener.h"

#include "syntaxParser.h"


using namespace antlrcpp;

using namespace antlr4;

namespace {

struct SyntaxParserStaticData final {
  SyntaxParserStaticData(std::vector<std::string> ruleNames,
                        std::vector<std::string> literalNames,
                        std::vector<std::string> symbolicNames)
      : ruleNames(std::move(ruleNames)), literalNames(std::move(literalNames)),
        symbolicNames(std::move(symbolicNames)),
        vocabulary(this->literalNames, this->symbolicNames) {}

  SyntaxParserStaticData(const SyntaxParserStaticData&) = delete;
  SyntaxParserStaticData(SyntaxParserStaticData&&) = delete;
  SyntaxParserStaticData& operator=(const SyntaxParserStaticData&) = delete;
  SyntaxParserStaticData& operator=(SyntaxParserStaticData&&) = delete;

  std::vector<antlr4::dfa::DFA> decisionToDFA;
  antlr4::atn::PredictionContextCache sharedContextCache;
  const std::vector<std::string> ruleNames;
  const std::vector<std::string> literalNames;
  const std::vector<std::string> symbolicNames;
  const antlr4::dfa::Vocabulary vocabulary;
  antlr4::atn::SerializedATNView serializedATN;
  std::unique_ptr<antlr4::atn::ATN> atn;
};

::antlr4::internal::OnceFlag syntaxParserOnceFlag;
#if ANTLR4_USE_THREAD_LOCAL_CACHE
static thread_local
#endif
std::unique_ptr<SyntaxParserStaticData> syntaxParserStaticData = nullptr;

void syntaxParserInitialize() {
#if ANTLR4_USE_THREAD_LOCAL_CACHE
  if (syntaxParserStaticData != nullptr) {
    return;
  }
#else
  assert(syntaxParserStaticData == nullptr);
#endif
  auto staticData = std::make_unique<SyntaxParserStaticData>(
    std::vector<std::string>{
      "program", "statementList", "statement", "declStmt", "inputStmt", 
      "outputStmt", "ifStmt", "elseBlock", "forStmt", "funcDefStmt", "returnStmt", 
      "exprStmt", "block", "parameterList", "expressionList", "condition", 
      "comparison", "expression", "term", "factor", "postfixOp", "primary", 
      "functionCall", "vector", "arrayAccess", "comparisonOp", "logicalOp"
    },
    std::vector<std::string>{
      "", "'program'", "'end'", "'let'", "'read'", "'say'", "'if'", "'else'", 
      "'for'", "'fn'", "'return'", "'<.>'", "'//'", "'^^'", "'%%'", "'@@'", 
      "'=='", "'!='", "'>='", "'<='", "'|>'", "'+'", "'-'", "'*'", "'/'", 
      "'%'", "'>'", "'<'", "'&'", "'|'", "'^'", "'!'", "'~'", "'='", "'('", 
      "')'", "'{'", "'}'", "'['", "']'", "';'", "','"
    },
    std::vector<std::string>{
      "", "PROGRAM", "END", "LET", "READ", "SAY", "IF", "ELSE", "FOR", "FN", 
      "RETURN", "DOT", "FLOORDIV", "GCD", "LCM", "POWER", "EQ", "NEQ", "GTE", 
      "LTE", "RELU", "PLUS", "MINUS", "MULT", "DIV", "MOD", "GT", "LT", 
      "AND", "OR", "XOR", "FACTORIAL", "SIGMOID", "ASSIGN", "LPAREN", "RPAREN", 
      "LBRACE", "RBRACE", "LBRACKET", "RBRACKET", "SEMI", "COMMA", "NUMBER", 
      "STRING", "IDENTIFIER", "COMMENT", "WS"
    }
  );
  static const int32_t serializedATNSegment[] = {
  	4,1,46,231,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,2,5,7,5,2,6,7,6,2,
  	7,7,7,2,8,7,8,2,9,7,9,2,10,7,10,2,11,7,11,2,12,7,12,2,13,7,13,2,14,7,
  	14,2,15,7,15,2,16,7,16,2,17,7,17,2,18,7,18,2,19,7,19,2,20,7,20,2,21,7,
  	21,2,22,7,22,2,23,7,23,2,24,7,24,2,25,7,25,2,26,7,26,1,0,1,0,1,0,1,0,
  	1,0,1,1,5,1,61,8,1,10,1,12,1,64,9,1,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,3,
  	2,74,8,2,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,3,3,88,8,3,1,
  	4,1,4,1,4,1,4,1,5,1,5,1,5,1,5,1,6,1,6,1,6,1,6,1,6,1,6,3,6,104,8,6,1,7,
  	1,7,1,7,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,8,1,9,1,
  	9,1,9,1,9,3,9,127,8,9,1,9,1,9,1,9,1,10,1,10,1,10,1,10,1,11,1,11,1,11,
  	1,12,1,12,1,12,1,12,1,13,1,13,1,13,5,13,146,8,13,10,13,12,13,149,9,13,
  	1,14,1,14,1,14,5,14,154,8,14,10,14,12,14,157,9,14,1,15,1,15,1,15,1,15,
  	5,15,163,8,15,10,15,12,15,166,9,15,1,16,1,16,1,16,1,16,1,17,1,17,1,17,
  	5,17,175,8,17,10,17,12,17,178,9,17,1,18,1,18,1,18,5,18,183,8,18,10,18,
  	12,18,186,9,18,1,19,3,19,189,8,19,1,19,1,19,3,19,193,8,19,1,20,1,20,1,
  	21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,1,21,3,21,207,8,21,1,22,1,
  	22,1,22,3,22,212,8,22,1,22,1,22,1,23,1,23,3,23,218,8,23,1,23,1,23,1,24,
  	1,24,1,24,1,24,1,24,1,25,1,25,1,26,1,26,1,26,0,0,27,0,2,4,6,8,10,12,14,
  	16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,0,5,1,0,21,22,
  	2,0,11,15,23,25,2,0,20,20,31,32,2,0,16,19,26,27,1,0,28,30,229,0,54,1,
  	0,0,0,2,62,1,0,0,0,4,73,1,0,0,0,6,87,1,0,0,0,8,89,1,0,0,0,10,93,1,0,0,
  	0,12,97,1,0,0,0,14,105,1,0,0,0,16,108,1,0,0,0,18,122,1,0,0,0,20,131,1,
  	0,0,0,22,135,1,0,0,0,24,138,1,0,0,0,26,142,1,0,0,0,28,150,1,0,0,0,30,
  	158,1,0,0,0,32,167,1,0,0,0,34,171,1,0,0,0,36,179,1,0,0,0,38,188,1,0,0,
  	0,40,194,1,0,0,0,42,206,1,0,0,0,44,208,1,0,0,0,46,215,1,0,0,0,48,221,
  	1,0,0,0,50,226,1,0,0,0,52,228,1,0,0,0,54,55,5,1,0,0,55,56,3,2,1,0,56,
  	57,5,2,0,0,57,58,5,0,0,1,58,1,1,0,0,0,59,61,3,4,2,0,60,59,1,0,0,0,61,
  	64,1,0,0,0,62,60,1,0,0,0,62,63,1,0,0,0,63,3,1,0,0,0,64,62,1,0,0,0,65,
  	74,3,6,3,0,66,74,3,8,4,0,67,74,3,10,5,0,68,74,3,12,6,0,69,74,3,16,8,0,
  	70,74,3,18,9,0,71,74,3,20,10,0,72,74,3,22,11,0,73,65,1,0,0,0,73,66,1,
  	0,0,0,73,67,1,0,0,0,73,68,1,0,0,0,73,69,1,0,0,0,73,70,1,0,0,0,73,71,1,
  	0,0,0,73,72,1,0,0,0,74,5,1,0,0,0,75,76,5,3,0,0,76,77,5,44,0,0,77,78,5,
  	33,0,0,78,79,3,34,17,0,79,80,5,40,0,0,80,88,1,0,0,0,81,82,5,3,0,0,82,
  	83,3,48,24,0,83,84,5,33,0,0,84,85,3,34,17,0,85,86,5,40,0,0,86,88,1,0,
  	0,0,87,75,1,0,0,0,87,81,1,0,0,0,88,7,1,0,0,0,89,90,5,4,0,0,90,91,5,44,
  	0,0,91,92,5,40,0,0,92,9,1,0,0,0,93,94,5,5,0,0,94,95,3,28,14,0,95,96,5,
  	40,0,0,96,11,1,0,0,0,97,98,5,6,0,0,98,99,5,34,0,0,99,100,3,30,15,0,100,
  	101,5,35,0,0,101,103,3,24,12,0,102,104,3,14,7,0,103,102,1,0,0,0,103,104,
  	1,0,0,0,104,13,1,0,0,0,105,106,5,7,0,0,106,107,3,24,12,0,107,15,1,0,0,
  	0,108,109,5,8,0,0,109,110,5,34,0,0,110,111,5,44,0,0,111,112,5,33,0,0,
  	112,113,3,34,17,0,113,114,5,40,0,0,114,115,3,30,15,0,115,116,5,40,0,0,
  	116,117,5,44,0,0,117,118,5,33,0,0,118,119,3,34,17,0,119,120,5,35,0,0,
  	120,121,3,24,12,0,121,17,1,0,0,0,122,123,5,9,0,0,123,124,5,44,0,0,124,
  	126,5,34,0,0,125,127,3,26,13,0,126,125,1,0,0,0,126,127,1,0,0,0,127,128,
  	1,0,0,0,128,129,5,35,0,0,129,130,3,24,12,0,130,19,1,0,0,0,131,132,5,10,
  	0,0,132,133,3,34,17,0,133,134,5,40,0,0,134,21,1,0,0,0,135,136,3,34,17,
  	0,136,137,5,40,0,0,137,23,1,0,0,0,138,139,5,36,0,0,139,140,3,2,1,0,140,
  	141,5,37,0,0,141,25,1,0,0,0,142,147,5,44,0,0,143,144,5,41,0,0,144,146,
  	5,44,0,0,145,143,1,0,0,0,146,149,1,0,0,0,147,145,1,0,0,0,147,148,1,0,
  	0,0,148,27,1,0,0,0,149,147,1,0,0,0,150,155,3,34,17,0,151,152,5,41,0,0,
  	152,154,3,34,17,0,153,151,1,0,0,0,154,157,1,0,0,0,155,153,1,0,0,0,155,
  	156,1,0,0,0,156,29,1,0,0,0,157,155,1,0,0,0,158,164,3,32,16,0,159,160,
  	3,52,26,0,160,161,3,32,16,0,161,163,1,0,0,0,162,159,1,0,0,0,163,166,1,
  	0,0,0,164,162,1,0,0,0,164,165,1,0,0,0,165,31,1,0,0,0,166,164,1,0,0,0,
  	167,168,3,34,17,0,168,169,3,50,25,0,169,170,3,34,17,0,170,33,1,0,0,0,
  	171,176,3,36,18,0,172,173,7,0,0,0,173,175,3,36,18,0,174,172,1,0,0,0,175,
  	178,1,0,0,0,176,174,1,0,0,0,176,177,1,0,0,0,177,35,1,0,0,0,178,176,1,
  	0,0,0,179,184,3,38,19,0,180,181,7,1,0,0,181,183,3,38,19,0,182,180,1,0,
  	0,0,183,186,1,0,0,0,184,182,1,0,0,0,184,185,1,0,0,0,185,37,1,0,0,0,186,
  	184,1,0,0,0,187,189,5,22,0,0,188,187,1,0,0,0,188,189,1,0,0,0,189,190,
  	1,0,0,0,190,192,3,42,21,0,191,193,3,40,20,0,192,191,1,0,0,0,192,193,1,
  	0,0,0,193,39,1,0,0,0,194,195,7,2,0,0,195,41,1,0,0,0,196,207,5,44,0,0,
  	197,207,5,42,0,0,198,207,5,43,0,0,199,200,5,34,0,0,200,201,3,34,17,0,
  	201,202,5,35,0,0,202,207,1,0,0,0,203,207,3,44,22,0,204,207,3,46,23,0,
  	205,207,3,48,24,0,206,196,1,0,0,0,206,197,1,0,0,0,206,198,1,0,0,0,206,
  	199,1,0,0,0,206,203,1,0,0,0,206,204,1,0,0,0,206,205,1,0,0,0,207,43,1,
  	0,0,0,208,209,5,44,0,0,209,211,5,34,0,0,210,212,3,28,14,0,211,210,1,0,
  	0,0,211,212,1,0,0,0,212,213,1,0,0,0,213,214,5,35,0,0,214,45,1,0,0,0,215,
  	217,5,38,0,0,216,218,3,28,14,0,217,216,1,0,0,0,217,218,1,0,0,0,218,219,
  	1,0,0,0,219,220,5,39,0,0,220,47,1,0,0,0,221,222,5,44,0,0,222,223,5,38,
  	0,0,223,224,3,34,17,0,224,225,5,39,0,0,225,49,1,0,0,0,226,227,7,3,0,0,
  	227,51,1,0,0,0,228,229,7,4,0,0,229,53,1,0,0,0,15,62,73,87,103,126,147,
  	155,164,176,184,188,192,206,211,217
  };
  staticData->serializedATN = antlr4::atn::SerializedATNView(serializedATNSegment, sizeof(serializedATNSegment) / sizeof(serializedATNSegment[0]));

  antlr4::atn::ATNDeserializer deserializer;
  staticData->atn = deserializer.deserialize(staticData->serializedATN);

  const size_t count = staticData->atn->getNumberOfDecisions();
  staticData->decisionToDFA.reserve(count);
  for (size_t i = 0; i < count; i++) { 
    staticData->decisionToDFA.emplace_back(staticData->atn->getDecisionState(i), i);
  }
  syntaxParserStaticData = std::move(staticData);
}

}

syntaxParser::syntaxParser(TokenStream *input) : syntaxParser(input, antlr4::atn::ParserATNSimulatorOptions()) {}

syntaxParser::syntaxParser(TokenStream *input, const antlr4::atn::ParserATNSimulatorOptions &options) : Parser(input) {
  syntaxParser::initialize();
  _interpreter = new atn::ParserATNSimulator(this, *syntaxParserStaticData->atn, syntaxParserStaticData->decisionToDFA, syntaxParserStaticData->sharedContextCache, options);
}

syntaxParser::~syntaxParser() {
  delete _interpreter;
}

const atn::ATN& syntaxParser::getATN() const {
  return *syntaxParserStaticData->atn;
}

std::string syntaxParser::getGrammarFileName() const {
  return "syntax.g4";
}

const std::vector<std::string>& syntaxParser::getRuleNames() const {
  return syntaxParserStaticData->ruleNames;
}

const dfa::Vocabulary& syntaxParser::getVocabulary() const {
  return syntaxParserStaticData->vocabulary;
}

antlr4::atn::SerializedATNView syntaxParser::getSerializedATN() const {
  return syntaxParserStaticData->serializedATN;
}


//----------------- ProgramContext ------------------------------------------------------------------

syntaxParser::ProgramContext::ProgramContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ProgramContext::PROGRAM() {
  return getToken(syntaxParser::PROGRAM, 0);
}

syntaxParser::StatementListContext* syntaxParser::ProgramContext::statementList() {
  return getRuleContext<syntaxParser::StatementListContext>(0);
}

tree::TerminalNode* syntaxParser::ProgramContext::END() {
  return getToken(syntaxParser::END, 0);
}

tree::TerminalNode* syntaxParser::ProgramContext::EOF() {
  return getToken(syntaxParser::EOF, 0);
}


size_t syntaxParser::ProgramContext::getRuleIndex() const {
  return syntaxParser::RuleProgram;
}

void syntaxParser::ProgramContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterProgram(this);
}

void syntaxParser::ProgramContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitProgram(this);
}

syntaxParser::ProgramContext* syntaxParser::program() {
  ProgramContext *_localctx = _tracker.createInstance<ProgramContext>(_ctx, getState());
  enterRule(_localctx, 0, syntaxParser::RuleProgram);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(54);
    match(syntaxParser::PROGRAM);
    setState(55);
    statementList();
    setState(56);
    match(syntaxParser::END);
    setState(57);
    match(syntaxParser::EOF);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- StatementListContext ------------------------------------------------------------------

syntaxParser::StatementListContext::StatementListContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::StatementContext *> syntaxParser::StatementListContext::statement() {
  return getRuleContexts<syntaxParser::StatementContext>();
}

syntaxParser::StatementContext* syntaxParser::StatementListContext::statement(size_t i) {
  return getRuleContext<syntaxParser::StatementContext>(i);
}


size_t syntaxParser::StatementListContext::getRuleIndex() const {
  return syntaxParser::RuleStatementList;
}

void syntaxParser::StatementListContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterStatementList(this);
}

void syntaxParser::StatementListContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitStatementList(this);
}

syntaxParser::StatementListContext* syntaxParser::statementList() {
  StatementListContext *_localctx = _tracker.createInstance<StatementListContext>(_ctx, getState());
  enterRule(_localctx, 2, syntaxParser::RuleStatementList);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(62);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 31078387550072) != 0)) {
      setState(59);
      statement();
      setState(64);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- StatementContext ------------------------------------------------------------------

syntaxParser::StatementContext::StatementContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

syntaxParser::DeclStmtContext* syntaxParser::StatementContext::declStmt() {
  return getRuleContext<syntaxParser::DeclStmtContext>(0);
}

syntaxParser::InputStmtContext* syntaxParser::StatementContext::inputStmt() {
  return getRuleContext<syntaxParser::InputStmtContext>(0);
}

syntaxParser::OutputStmtContext* syntaxParser::StatementContext::outputStmt() {
  return getRuleContext<syntaxParser::OutputStmtContext>(0);
}

syntaxParser::IfStmtContext* syntaxParser::StatementContext::ifStmt() {
  return getRuleContext<syntaxParser::IfStmtContext>(0);
}

syntaxParser::ForStmtContext* syntaxParser::StatementContext::forStmt() {
  return getRuleContext<syntaxParser::ForStmtContext>(0);
}

syntaxParser::FuncDefStmtContext* syntaxParser::StatementContext::funcDefStmt() {
  return getRuleContext<syntaxParser::FuncDefStmtContext>(0);
}

syntaxParser::ReturnStmtContext* syntaxParser::StatementContext::returnStmt() {
  return getRuleContext<syntaxParser::ReturnStmtContext>(0);
}

syntaxParser::ExprStmtContext* syntaxParser::StatementContext::exprStmt() {
  return getRuleContext<syntaxParser::ExprStmtContext>(0);
}


size_t syntaxParser::StatementContext::getRuleIndex() const {
  return syntaxParser::RuleStatement;
}

void syntaxParser::StatementContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterStatement(this);
}

void syntaxParser::StatementContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitStatement(this);
}

syntaxParser::StatementContext* syntaxParser::statement() {
  StatementContext *_localctx = _tracker.createInstance<StatementContext>(_ctx, getState());
  enterRule(_localctx, 4, syntaxParser::RuleStatement);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    setState(73);
    _errHandler->sync(this);
    switch (_input->LA(1)) {
      case syntaxParser::LET: {
        enterOuterAlt(_localctx, 1);
        setState(65);
        declStmt();
        break;
      }

      case syntaxParser::READ: {
        enterOuterAlt(_localctx, 2);
        setState(66);
        inputStmt();
        break;
      }

      case syntaxParser::SAY: {
        enterOuterAlt(_localctx, 3);
        setState(67);
        outputStmt();
        break;
      }

      case syntaxParser::IF: {
        enterOuterAlt(_localctx, 4);
        setState(68);
        ifStmt();
        break;
      }

      case syntaxParser::FOR: {
        enterOuterAlt(_localctx, 5);
        setState(69);
        forStmt();
        break;
      }

      case syntaxParser::FN: {
        enterOuterAlt(_localctx, 6);
        setState(70);
        funcDefStmt();
        break;
      }

      case syntaxParser::RETURN: {
        enterOuterAlt(_localctx, 7);
        setState(71);
        returnStmt();
        break;
      }

      case syntaxParser::MINUS:
      case syntaxParser::LPAREN:
      case syntaxParser::LBRACKET:
      case syntaxParser::NUMBER:
      case syntaxParser::STRING:
      case syntaxParser::IDENTIFIER: {
        enterOuterAlt(_localctx, 8);
        setState(72);
        exprStmt();
        break;
      }

    default:
      throw NoViableAltException(this);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- DeclStmtContext ------------------------------------------------------------------

syntaxParser::DeclStmtContext::DeclStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::DeclStmtContext::LET() {
  return getToken(syntaxParser::LET, 0);
}

tree::TerminalNode* syntaxParser::DeclStmtContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::DeclStmtContext::ASSIGN() {
  return getToken(syntaxParser::ASSIGN, 0);
}

syntaxParser::ExpressionContext* syntaxParser::DeclStmtContext::expression() {
  return getRuleContext<syntaxParser::ExpressionContext>(0);
}

tree::TerminalNode* syntaxParser::DeclStmtContext::SEMI() {
  return getToken(syntaxParser::SEMI, 0);
}

syntaxParser::ArrayAccessContext* syntaxParser::DeclStmtContext::arrayAccess() {
  return getRuleContext<syntaxParser::ArrayAccessContext>(0);
}


size_t syntaxParser::DeclStmtContext::getRuleIndex() const {
  return syntaxParser::RuleDeclStmt;
}

void syntaxParser::DeclStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterDeclStmt(this);
}

void syntaxParser::DeclStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitDeclStmt(this);
}

syntaxParser::DeclStmtContext* syntaxParser::declStmt() {
  DeclStmtContext *_localctx = _tracker.createInstance<DeclStmtContext>(_ctx, getState());
  enterRule(_localctx, 6, syntaxParser::RuleDeclStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    setState(87);
    _errHandler->sync(this);
    switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 2, _ctx)) {
    case 1: {
      enterOuterAlt(_localctx, 1);
      setState(75);
      match(syntaxParser::LET);
      setState(76);
      match(syntaxParser::IDENTIFIER);
      setState(77);
      match(syntaxParser::ASSIGN);
      setState(78);
      expression();
      setState(79);
      match(syntaxParser::SEMI);
      break;
    }

    case 2: {
      enterOuterAlt(_localctx, 2);
      setState(81);
      match(syntaxParser::LET);
      setState(82);
      arrayAccess();
      setState(83);
      match(syntaxParser::ASSIGN);
      setState(84);
      expression();
      setState(85);
      match(syntaxParser::SEMI);
      break;
    }

    default:
      break;
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- InputStmtContext ------------------------------------------------------------------

syntaxParser::InputStmtContext::InputStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::InputStmtContext::READ() {
  return getToken(syntaxParser::READ, 0);
}

tree::TerminalNode* syntaxParser::InputStmtContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::InputStmtContext::SEMI() {
  return getToken(syntaxParser::SEMI, 0);
}


size_t syntaxParser::InputStmtContext::getRuleIndex() const {
  return syntaxParser::RuleInputStmt;
}

void syntaxParser::InputStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterInputStmt(this);
}

void syntaxParser::InputStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitInputStmt(this);
}

syntaxParser::InputStmtContext* syntaxParser::inputStmt() {
  InputStmtContext *_localctx = _tracker.createInstance<InputStmtContext>(_ctx, getState());
  enterRule(_localctx, 8, syntaxParser::RuleInputStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(89);
    match(syntaxParser::READ);
    setState(90);
    match(syntaxParser::IDENTIFIER);
    setState(91);
    match(syntaxParser::SEMI);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- OutputStmtContext ------------------------------------------------------------------

syntaxParser::OutputStmtContext::OutputStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::OutputStmtContext::SAY() {
  return getToken(syntaxParser::SAY, 0);
}

syntaxParser::ExpressionListContext* syntaxParser::OutputStmtContext::expressionList() {
  return getRuleContext<syntaxParser::ExpressionListContext>(0);
}

tree::TerminalNode* syntaxParser::OutputStmtContext::SEMI() {
  return getToken(syntaxParser::SEMI, 0);
}


size_t syntaxParser::OutputStmtContext::getRuleIndex() const {
  return syntaxParser::RuleOutputStmt;
}

void syntaxParser::OutputStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterOutputStmt(this);
}

void syntaxParser::OutputStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitOutputStmt(this);
}

syntaxParser::OutputStmtContext* syntaxParser::outputStmt() {
  OutputStmtContext *_localctx = _tracker.createInstance<OutputStmtContext>(_ctx, getState());
  enterRule(_localctx, 10, syntaxParser::RuleOutputStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(93);
    match(syntaxParser::SAY);
    setState(94);
    expressionList();
    setState(95);
    match(syntaxParser::SEMI);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- IfStmtContext ------------------------------------------------------------------

syntaxParser::IfStmtContext::IfStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::IfStmtContext::IF() {
  return getToken(syntaxParser::IF, 0);
}

tree::TerminalNode* syntaxParser::IfStmtContext::LPAREN() {
  return getToken(syntaxParser::LPAREN, 0);
}

syntaxParser::ConditionContext* syntaxParser::IfStmtContext::condition() {
  return getRuleContext<syntaxParser::ConditionContext>(0);
}

tree::TerminalNode* syntaxParser::IfStmtContext::RPAREN() {
  return getToken(syntaxParser::RPAREN, 0);
}

syntaxParser::BlockContext* syntaxParser::IfStmtContext::block() {
  return getRuleContext<syntaxParser::BlockContext>(0);
}

syntaxParser::ElseBlockContext* syntaxParser::IfStmtContext::elseBlock() {
  return getRuleContext<syntaxParser::ElseBlockContext>(0);
}


size_t syntaxParser::IfStmtContext::getRuleIndex() const {
  return syntaxParser::RuleIfStmt;
}

void syntaxParser::IfStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterIfStmt(this);
}

void syntaxParser::IfStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitIfStmt(this);
}

syntaxParser::IfStmtContext* syntaxParser::ifStmt() {
  IfStmtContext *_localctx = _tracker.createInstance<IfStmtContext>(_ctx, getState());
  enterRule(_localctx, 12, syntaxParser::RuleIfStmt);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(97);
    match(syntaxParser::IF);
    setState(98);
    match(syntaxParser::LPAREN);
    setState(99);
    condition();
    setState(100);
    match(syntaxParser::RPAREN);
    setState(101);
    block();
    setState(103);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if (_la == syntaxParser::ELSE) {
      setState(102);
      elseBlock();
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ElseBlockContext ------------------------------------------------------------------

syntaxParser::ElseBlockContext::ElseBlockContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ElseBlockContext::ELSE() {
  return getToken(syntaxParser::ELSE, 0);
}

syntaxParser::BlockContext* syntaxParser::ElseBlockContext::block() {
  return getRuleContext<syntaxParser::BlockContext>(0);
}


size_t syntaxParser::ElseBlockContext::getRuleIndex() const {
  return syntaxParser::RuleElseBlock;
}

void syntaxParser::ElseBlockContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterElseBlock(this);
}

void syntaxParser::ElseBlockContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitElseBlock(this);
}

syntaxParser::ElseBlockContext* syntaxParser::elseBlock() {
  ElseBlockContext *_localctx = _tracker.createInstance<ElseBlockContext>(_ctx, getState());
  enterRule(_localctx, 14, syntaxParser::RuleElseBlock);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(105);
    match(syntaxParser::ELSE);
    setState(106);
    block();
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ForStmtContext ------------------------------------------------------------------

syntaxParser::ForStmtContext::ForStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ForStmtContext::FOR() {
  return getToken(syntaxParser::FOR, 0);
}

tree::TerminalNode* syntaxParser::ForStmtContext::LPAREN() {
  return getToken(syntaxParser::LPAREN, 0);
}

std::vector<tree::TerminalNode *> syntaxParser::ForStmtContext::IDENTIFIER() {
  return getTokens(syntaxParser::IDENTIFIER);
}

tree::TerminalNode* syntaxParser::ForStmtContext::IDENTIFIER(size_t i) {
  return getToken(syntaxParser::IDENTIFIER, i);
}

std::vector<tree::TerminalNode *> syntaxParser::ForStmtContext::ASSIGN() {
  return getTokens(syntaxParser::ASSIGN);
}

tree::TerminalNode* syntaxParser::ForStmtContext::ASSIGN(size_t i) {
  return getToken(syntaxParser::ASSIGN, i);
}

std::vector<syntaxParser::ExpressionContext *> syntaxParser::ForStmtContext::expression() {
  return getRuleContexts<syntaxParser::ExpressionContext>();
}

syntaxParser::ExpressionContext* syntaxParser::ForStmtContext::expression(size_t i) {
  return getRuleContext<syntaxParser::ExpressionContext>(i);
}

std::vector<tree::TerminalNode *> syntaxParser::ForStmtContext::SEMI() {
  return getTokens(syntaxParser::SEMI);
}

tree::TerminalNode* syntaxParser::ForStmtContext::SEMI(size_t i) {
  return getToken(syntaxParser::SEMI, i);
}

syntaxParser::ConditionContext* syntaxParser::ForStmtContext::condition() {
  return getRuleContext<syntaxParser::ConditionContext>(0);
}

tree::TerminalNode* syntaxParser::ForStmtContext::RPAREN() {
  return getToken(syntaxParser::RPAREN, 0);
}

syntaxParser::BlockContext* syntaxParser::ForStmtContext::block() {
  return getRuleContext<syntaxParser::BlockContext>(0);
}


size_t syntaxParser::ForStmtContext::getRuleIndex() const {
  return syntaxParser::RuleForStmt;
}

void syntaxParser::ForStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterForStmt(this);
}

void syntaxParser::ForStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitForStmt(this);
}

syntaxParser::ForStmtContext* syntaxParser::forStmt() {
  ForStmtContext *_localctx = _tracker.createInstance<ForStmtContext>(_ctx, getState());
  enterRule(_localctx, 16, syntaxParser::RuleForStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(108);
    match(syntaxParser::FOR);
    setState(109);
    match(syntaxParser::LPAREN);
    setState(110);
    match(syntaxParser::IDENTIFIER);
    setState(111);
    match(syntaxParser::ASSIGN);
    setState(112);
    expression();
    setState(113);
    match(syntaxParser::SEMI);
    setState(114);
    condition();
    setState(115);
    match(syntaxParser::SEMI);
    setState(116);
    match(syntaxParser::IDENTIFIER);
    setState(117);
    match(syntaxParser::ASSIGN);
    setState(118);
    expression();
    setState(119);
    match(syntaxParser::RPAREN);
    setState(120);
    block();
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- FuncDefStmtContext ------------------------------------------------------------------

syntaxParser::FuncDefStmtContext::FuncDefStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::FuncDefStmtContext::FN() {
  return getToken(syntaxParser::FN, 0);
}

tree::TerminalNode* syntaxParser::FuncDefStmtContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::FuncDefStmtContext::LPAREN() {
  return getToken(syntaxParser::LPAREN, 0);
}

tree::TerminalNode* syntaxParser::FuncDefStmtContext::RPAREN() {
  return getToken(syntaxParser::RPAREN, 0);
}

syntaxParser::BlockContext* syntaxParser::FuncDefStmtContext::block() {
  return getRuleContext<syntaxParser::BlockContext>(0);
}

syntaxParser::ParameterListContext* syntaxParser::FuncDefStmtContext::parameterList() {
  return getRuleContext<syntaxParser::ParameterListContext>(0);
}


size_t syntaxParser::FuncDefStmtContext::getRuleIndex() const {
  return syntaxParser::RuleFuncDefStmt;
}

void syntaxParser::FuncDefStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterFuncDefStmt(this);
}

void syntaxParser::FuncDefStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitFuncDefStmt(this);
}

syntaxParser::FuncDefStmtContext* syntaxParser::funcDefStmt() {
  FuncDefStmtContext *_localctx = _tracker.createInstance<FuncDefStmtContext>(_ctx, getState());
  enterRule(_localctx, 18, syntaxParser::RuleFuncDefStmt);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(122);
    match(syntaxParser::FN);
    setState(123);
    match(syntaxParser::IDENTIFIER);
    setState(124);
    match(syntaxParser::LPAREN);
    setState(126);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if (_la == syntaxParser::IDENTIFIER) {
      setState(125);
      parameterList();
    }
    setState(128);
    match(syntaxParser::RPAREN);
    setState(129);
    block();
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ReturnStmtContext ------------------------------------------------------------------

syntaxParser::ReturnStmtContext::ReturnStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ReturnStmtContext::RETURN() {
  return getToken(syntaxParser::RETURN, 0);
}

syntaxParser::ExpressionContext* syntaxParser::ReturnStmtContext::expression() {
  return getRuleContext<syntaxParser::ExpressionContext>(0);
}

tree::TerminalNode* syntaxParser::ReturnStmtContext::SEMI() {
  return getToken(syntaxParser::SEMI, 0);
}


size_t syntaxParser::ReturnStmtContext::getRuleIndex() const {
  return syntaxParser::RuleReturnStmt;
}

void syntaxParser::ReturnStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterReturnStmt(this);
}

void syntaxParser::ReturnStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitReturnStmt(this);
}

syntaxParser::ReturnStmtContext* syntaxParser::returnStmt() {
  ReturnStmtContext *_localctx = _tracker.createInstance<ReturnStmtContext>(_ctx, getState());
  enterRule(_localctx, 20, syntaxParser::RuleReturnStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(131);
    match(syntaxParser::RETURN);
    setState(132);
    expression();
    setState(133);
    match(syntaxParser::SEMI);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ExprStmtContext ------------------------------------------------------------------

syntaxParser::ExprStmtContext::ExprStmtContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

syntaxParser::ExpressionContext* syntaxParser::ExprStmtContext::expression() {
  return getRuleContext<syntaxParser::ExpressionContext>(0);
}

tree::TerminalNode* syntaxParser::ExprStmtContext::SEMI() {
  return getToken(syntaxParser::SEMI, 0);
}


size_t syntaxParser::ExprStmtContext::getRuleIndex() const {
  return syntaxParser::RuleExprStmt;
}

void syntaxParser::ExprStmtContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterExprStmt(this);
}

void syntaxParser::ExprStmtContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitExprStmt(this);
}

syntaxParser::ExprStmtContext* syntaxParser::exprStmt() {
  ExprStmtContext *_localctx = _tracker.createInstance<ExprStmtContext>(_ctx, getState());
  enterRule(_localctx, 22, syntaxParser::RuleExprStmt);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(135);
    expression();
    setState(136);
    match(syntaxParser::SEMI);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- BlockContext ------------------------------------------------------------------

syntaxParser::BlockContext::BlockContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::BlockContext::LBRACE() {
  return getToken(syntaxParser::LBRACE, 0);
}

syntaxParser::StatementListContext* syntaxParser::BlockContext::statementList() {
  return getRuleContext<syntaxParser::StatementListContext>(0);
}

tree::TerminalNode* syntaxParser::BlockContext::RBRACE() {
  return getToken(syntaxParser::RBRACE, 0);
}


size_t syntaxParser::BlockContext::getRuleIndex() const {
  return syntaxParser::RuleBlock;
}

void syntaxParser::BlockContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterBlock(this);
}

void syntaxParser::BlockContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitBlock(this);
}

syntaxParser::BlockContext* syntaxParser::block() {
  BlockContext *_localctx = _tracker.createInstance<BlockContext>(_ctx, getState());
  enterRule(_localctx, 24, syntaxParser::RuleBlock);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(138);
    match(syntaxParser::LBRACE);
    setState(139);
    statementList();
    setState(140);
    match(syntaxParser::RBRACE);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ParameterListContext ------------------------------------------------------------------

syntaxParser::ParameterListContext::ParameterListContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<tree::TerminalNode *> syntaxParser::ParameterListContext::IDENTIFIER() {
  return getTokens(syntaxParser::IDENTIFIER);
}

tree::TerminalNode* syntaxParser::ParameterListContext::IDENTIFIER(size_t i) {
  return getToken(syntaxParser::IDENTIFIER, i);
}

std::vector<tree::TerminalNode *> syntaxParser::ParameterListContext::COMMA() {
  return getTokens(syntaxParser::COMMA);
}

tree::TerminalNode* syntaxParser::ParameterListContext::COMMA(size_t i) {
  return getToken(syntaxParser::COMMA, i);
}


size_t syntaxParser::ParameterListContext::getRuleIndex() const {
  return syntaxParser::RuleParameterList;
}

void syntaxParser::ParameterListContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterParameterList(this);
}

void syntaxParser::ParameterListContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitParameterList(this);
}

syntaxParser::ParameterListContext* syntaxParser::parameterList() {
  ParameterListContext *_localctx = _tracker.createInstance<ParameterListContext>(_ctx, getState());
  enterRule(_localctx, 26, syntaxParser::RuleParameterList);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(142);
    match(syntaxParser::IDENTIFIER);
    setState(147);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while (_la == syntaxParser::COMMA) {
      setState(143);
      match(syntaxParser::COMMA);
      setState(144);
      match(syntaxParser::IDENTIFIER);
      setState(149);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ExpressionListContext ------------------------------------------------------------------

syntaxParser::ExpressionListContext::ExpressionListContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::ExpressionContext *> syntaxParser::ExpressionListContext::expression() {
  return getRuleContexts<syntaxParser::ExpressionContext>();
}

syntaxParser::ExpressionContext* syntaxParser::ExpressionListContext::expression(size_t i) {
  return getRuleContext<syntaxParser::ExpressionContext>(i);
}

std::vector<tree::TerminalNode *> syntaxParser::ExpressionListContext::COMMA() {
  return getTokens(syntaxParser::COMMA);
}

tree::TerminalNode* syntaxParser::ExpressionListContext::COMMA(size_t i) {
  return getToken(syntaxParser::COMMA, i);
}


size_t syntaxParser::ExpressionListContext::getRuleIndex() const {
  return syntaxParser::RuleExpressionList;
}

void syntaxParser::ExpressionListContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterExpressionList(this);
}

void syntaxParser::ExpressionListContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitExpressionList(this);
}

syntaxParser::ExpressionListContext* syntaxParser::expressionList() {
  ExpressionListContext *_localctx = _tracker.createInstance<ExpressionListContext>(_ctx, getState());
  enterRule(_localctx, 28, syntaxParser::RuleExpressionList);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(150);
    expression();
    setState(155);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while (_la == syntaxParser::COMMA) {
      setState(151);
      match(syntaxParser::COMMA);
      setState(152);
      expression();
      setState(157);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ConditionContext ------------------------------------------------------------------

syntaxParser::ConditionContext::ConditionContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::ComparisonContext *> syntaxParser::ConditionContext::comparison() {
  return getRuleContexts<syntaxParser::ComparisonContext>();
}

syntaxParser::ComparisonContext* syntaxParser::ConditionContext::comparison(size_t i) {
  return getRuleContext<syntaxParser::ComparisonContext>(i);
}

std::vector<syntaxParser::LogicalOpContext *> syntaxParser::ConditionContext::logicalOp() {
  return getRuleContexts<syntaxParser::LogicalOpContext>();
}

syntaxParser::LogicalOpContext* syntaxParser::ConditionContext::logicalOp(size_t i) {
  return getRuleContext<syntaxParser::LogicalOpContext>(i);
}


size_t syntaxParser::ConditionContext::getRuleIndex() const {
  return syntaxParser::RuleCondition;
}

void syntaxParser::ConditionContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterCondition(this);
}

void syntaxParser::ConditionContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitCondition(this);
}

syntaxParser::ConditionContext* syntaxParser::condition() {
  ConditionContext *_localctx = _tracker.createInstance<ConditionContext>(_ctx, getState());
  enterRule(_localctx, 30, syntaxParser::RuleCondition);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(158);
    comparison();
    setState(164);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 1879048192) != 0)) {
      setState(159);
      logicalOp();
      setState(160);
      comparison();
      setState(166);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ComparisonContext ------------------------------------------------------------------

syntaxParser::ComparisonContext::ComparisonContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::ExpressionContext *> syntaxParser::ComparisonContext::expression() {
  return getRuleContexts<syntaxParser::ExpressionContext>();
}

syntaxParser::ExpressionContext* syntaxParser::ComparisonContext::expression(size_t i) {
  return getRuleContext<syntaxParser::ExpressionContext>(i);
}

syntaxParser::ComparisonOpContext* syntaxParser::ComparisonContext::comparisonOp() {
  return getRuleContext<syntaxParser::ComparisonOpContext>(0);
}


size_t syntaxParser::ComparisonContext::getRuleIndex() const {
  return syntaxParser::RuleComparison;
}

void syntaxParser::ComparisonContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterComparison(this);
}

void syntaxParser::ComparisonContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitComparison(this);
}

syntaxParser::ComparisonContext* syntaxParser::comparison() {
  ComparisonContext *_localctx = _tracker.createInstance<ComparisonContext>(_ctx, getState());
  enterRule(_localctx, 32, syntaxParser::RuleComparison);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(167);
    expression();
    setState(168);
    comparisonOp();
    setState(169);
    expression();
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ExpressionContext ------------------------------------------------------------------

syntaxParser::ExpressionContext::ExpressionContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::TermContext *> syntaxParser::ExpressionContext::term() {
  return getRuleContexts<syntaxParser::TermContext>();
}

syntaxParser::TermContext* syntaxParser::ExpressionContext::term(size_t i) {
  return getRuleContext<syntaxParser::TermContext>(i);
}

std::vector<tree::TerminalNode *> syntaxParser::ExpressionContext::PLUS() {
  return getTokens(syntaxParser::PLUS);
}

tree::TerminalNode* syntaxParser::ExpressionContext::PLUS(size_t i) {
  return getToken(syntaxParser::PLUS, i);
}

std::vector<tree::TerminalNode *> syntaxParser::ExpressionContext::MINUS() {
  return getTokens(syntaxParser::MINUS);
}

tree::TerminalNode* syntaxParser::ExpressionContext::MINUS(size_t i) {
  return getToken(syntaxParser::MINUS, i);
}


size_t syntaxParser::ExpressionContext::getRuleIndex() const {
  return syntaxParser::RuleExpression;
}

void syntaxParser::ExpressionContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterExpression(this);
}

void syntaxParser::ExpressionContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitExpression(this);
}

syntaxParser::ExpressionContext* syntaxParser::expression() {
  ExpressionContext *_localctx = _tracker.createInstance<ExpressionContext>(_ctx, getState());
  enterRule(_localctx, 34, syntaxParser::RuleExpression);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(171);
    term();
    setState(176);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while (_la == syntaxParser::PLUS

    || _la == syntaxParser::MINUS) {
      setState(172);
      _la = _input->LA(1);
      if (!(_la == syntaxParser::PLUS

      || _la == syntaxParser::MINUS)) {
      _errHandler->recoverInline(this);
      }
      else {
        _errHandler->reportMatch(this);
        consume();
      }
      setState(173);
      term();
      setState(178);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- TermContext ------------------------------------------------------------------

syntaxParser::TermContext::TermContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

std::vector<syntaxParser::FactorContext *> syntaxParser::TermContext::factor() {
  return getRuleContexts<syntaxParser::FactorContext>();
}

syntaxParser::FactorContext* syntaxParser::TermContext::factor(size_t i) {
  return getRuleContext<syntaxParser::FactorContext>(i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::MULT() {
  return getTokens(syntaxParser::MULT);
}

tree::TerminalNode* syntaxParser::TermContext::MULT(size_t i) {
  return getToken(syntaxParser::MULT, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::DIV() {
  return getTokens(syntaxParser::DIV);
}

tree::TerminalNode* syntaxParser::TermContext::DIV(size_t i) {
  return getToken(syntaxParser::DIV, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::FLOORDIV() {
  return getTokens(syntaxParser::FLOORDIV);
}

tree::TerminalNode* syntaxParser::TermContext::FLOORDIV(size_t i) {
  return getToken(syntaxParser::FLOORDIV, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::MOD() {
  return getTokens(syntaxParser::MOD);
}

tree::TerminalNode* syntaxParser::TermContext::MOD(size_t i) {
  return getToken(syntaxParser::MOD, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::GCD() {
  return getTokens(syntaxParser::GCD);
}

tree::TerminalNode* syntaxParser::TermContext::GCD(size_t i) {
  return getToken(syntaxParser::GCD, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::LCM() {
  return getTokens(syntaxParser::LCM);
}

tree::TerminalNode* syntaxParser::TermContext::LCM(size_t i) {
  return getToken(syntaxParser::LCM, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::POWER() {
  return getTokens(syntaxParser::POWER);
}

tree::TerminalNode* syntaxParser::TermContext::POWER(size_t i) {
  return getToken(syntaxParser::POWER, i);
}

std::vector<tree::TerminalNode *> syntaxParser::TermContext::DOT() {
  return getTokens(syntaxParser::DOT);
}

tree::TerminalNode* syntaxParser::TermContext::DOT(size_t i) {
  return getToken(syntaxParser::DOT, i);
}


size_t syntaxParser::TermContext::getRuleIndex() const {
  return syntaxParser::RuleTerm;
}

void syntaxParser::TermContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterTerm(this);
}

void syntaxParser::TermContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitTerm(this);
}

syntaxParser::TermContext* syntaxParser::term() {
  TermContext *_localctx = _tracker.createInstance<TermContext>(_ctx, getState());
  enterRule(_localctx, 36, syntaxParser::RuleTerm);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(179);
    factor();
    setState(184);
    _errHandler->sync(this);
    _la = _input->LA(1);
    while ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 58783744) != 0)) {
      setState(180);
      _la = _input->LA(1);
      if (!((((_la & ~ 0x3fULL) == 0) &&
        ((1ULL << _la) & 58783744) != 0))) {
      _errHandler->recoverInline(this);
      }
      else {
        _errHandler->reportMatch(this);
        consume();
      }
      setState(181);
      factor();
      setState(186);
      _errHandler->sync(this);
      _la = _input->LA(1);
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- FactorContext ------------------------------------------------------------------

syntaxParser::FactorContext::FactorContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

syntaxParser::PrimaryContext* syntaxParser::FactorContext::primary() {
  return getRuleContext<syntaxParser::PrimaryContext>(0);
}

tree::TerminalNode* syntaxParser::FactorContext::MINUS() {
  return getToken(syntaxParser::MINUS, 0);
}

syntaxParser::PostfixOpContext* syntaxParser::FactorContext::postfixOp() {
  return getRuleContext<syntaxParser::PostfixOpContext>(0);
}


size_t syntaxParser::FactorContext::getRuleIndex() const {
  return syntaxParser::RuleFactor;
}

void syntaxParser::FactorContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterFactor(this);
}

void syntaxParser::FactorContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitFactor(this);
}

syntaxParser::FactorContext* syntaxParser::factor() {
  FactorContext *_localctx = _tracker.createInstance<FactorContext>(_ctx, getState());
  enterRule(_localctx, 38, syntaxParser::RuleFactor);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(188);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if (_la == syntaxParser::MINUS) {
      setState(187);
      match(syntaxParser::MINUS);
    }
    setState(190);
    primary();
    setState(192);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 6443499520) != 0)) {
      setState(191);
      postfixOp();
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- PostfixOpContext ------------------------------------------------------------------

syntaxParser::PostfixOpContext::PostfixOpContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::PostfixOpContext::FACTORIAL() {
  return getToken(syntaxParser::FACTORIAL, 0);
}

tree::TerminalNode* syntaxParser::PostfixOpContext::SIGMOID() {
  return getToken(syntaxParser::SIGMOID, 0);
}

tree::TerminalNode* syntaxParser::PostfixOpContext::RELU() {
  return getToken(syntaxParser::RELU, 0);
}


size_t syntaxParser::PostfixOpContext::getRuleIndex() const {
  return syntaxParser::RulePostfixOp;
}

void syntaxParser::PostfixOpContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterPostfixOp(this);
}

void syntaxParser::PostfixOpContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitPostfixOp(this);
}

syntaxParser::PostfixOpContext* syntaxParser::postfixOp() {
  PostfixOpContext *_localctx = _tracker.createInstance<PostfixOpContext>(_ctx, getState());
  enterRule(_localctx, 40, syntaxParser::RulePostfixOp);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(194);
    _la = _input->LA(1);
    if (!((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 6443499520) != 0))) {
    _errHandler->recoverInline(this);
    }
    else {
      _errHandler->reportMatch(this);
      consume();
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- PrimaryContext ------------------------------------------------------------------

syntaxParser::PrimaryContext::PrimaryContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::PrimaryContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::PrimaryContext::NUMBER() {
  return getToken(syntaxParser::NUMBER, 0);
}

tree::TerminalNode* syntaxParser::PrimaryContext::STRING() {
  return getToken(syntaxParser::STRING, 0);
}

tree::TerminalNode* syntaxParser::PrimaryContext::LPAREN() {
  return getToken(syntaxParser::LPAREN, 0);
}

syntaxParser::ExpressionContext* syntaxParser::PrimaryContext::expression() {
  return getRuleContext<syntaxParser::ExpressionContext>(0);
}

tree::TerminalNode* syntaxParser::PrimaryContext::RPAREN() {
  return getToken(syntaxParser::RPAREN, 0);
}

syntaxParser::FunctionCallContext* syntaxParser::PrimaryContext::functionCall() {
  return getRuleContext<syntaxParser::FunctionCallContext>(0);
}

syntaxParser::VectorContext* syntaxParser::PrimaryContext::vector() {
  return getRuleContext<syntaxParser::VectorContext>(0);
}

syntaxParser::ArrayAccessContext* syntaxParser::PrimaryContext::arrayAccess() {
  return getRuleContext<syntaxParser::ArrayAccessContext>(0);
}


size_t syntaxParser::PrimaryContext::getRuleIndex() const {
  return syntaxParser::RulePrimary;
}

void syntaxParser::PrimaryContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterPrimary(this);
}

void syntaxParser::PrimaryContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitPrimary(this);
}

syntaxParser::PrimaryContext* syntaxParser::primary() {
  PrimaryContext *_localctx = _tracker.createInstance<PrimaryContext>(_ctx, getState());
  enterRule(_localctx, 42, syntaxParser::RulePrimary);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    setState(206);
    _errHandler->sync(this);
    switch (getInterpreter<atn::ParserATNSimulator>()->adaptivePredict(_input, 12, _ctx)) {
    case 1: {
      enterOuterAlt(_localctx, 1);
      setState(196);
      match(syntaxParser::IDENTIFIER);
      break;
    }

    case 2: {
      enterOuterAlt(_localctx, 2);
      setState(197);
      match(syntaxParser::NUMBER);
      break;
    }

    case 3: {
      enterOuterAlt(_localctx, 3);
      setState(198);
      match(syntaxParser::STRING);
      break;
    }

    case 4: {
      enterOuterAlt(_localctx, 4);
      setState(199);
      match(syntaxParser::LPAREN);
      setState(200);
      expression();
      setState(201);
      match(syntaxParser::RPAREN);
      break;
    }

    case 5: {
      enterOuterAlt(_localctx, 5);
      setState(203);
      functionCall();
      break;
    }

    case 6: {
      enterOuterAlt(_localctx, 6);
      setState(204);
      vector();
      break;
    }

    case 7: {
      enterOuterAlt(_localctx, 7);
      setState(205);
      arrayAccess();
      break;
    }

    default:
      break;
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- FunctionCallContext ------------------------------------------------------------------

syntaxParser::FunctionCallContext::FunctionCallContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::FunctionCallContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::FunctionCallContext::LPAREN() {
  return getToken(syntaxParser::LPAREN, 0);
}

tree::TerminalNode* syntaxParser::FunctionCallContext::RPAREN() {
  return getToken(syntaxParser::RPAREN, 0);
}

syntaxParser::ExpressionListContext* syntaxParser::FunctionCallContext::expressionList() {
  return getRuleContext<syntaxParser::ExpressionListContext>(0);
}


size_t syntaxParser::FunctionCallContext::getRuleIndex() const {
  return syntaxParser::RuleFunctionCall;
}

void syntaxParser::FunctionCallContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterFunctionCall(this);
}

void syntaxParser::FunctionCallContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitFunctionCall(this);
}

syntaxParser::FunctionCallContext* syntaxParser::functionCall() {
  FunctionCallContext *_localctx = _tracker.createInstance<FunctionCallContext>(_ctx, getState());
  enterRule(_localctx, 44, syntaxParser::RuleFunctionCall);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(208);
    match(syntaxParser::IDENTIFIER);
    setState(209);
    match(syntaxParser::LPAREN);
    setState(211);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 31078387548160) != 0)) {
      setState(210);
      expressionList();
    }
    setState(213);
    match(syntaxParser::RPAREN);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- VectorContext ------------------------------------------------------------------

syntaxParser::VectorContext::VectorContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::VectorContext::LBRACKET() {
  return getToken(syntaxParser::LBRACKET, 0);
}

tree::TerminalNode* syntaxParser::VectorContext::RBRACKET() {
  return getToken(syntaxParser::RBRACKET, 0);
}

syntaxParser::ExpressionListContext* syntaxParser::VectorContext::expressionList() {
  return getRuleContext<syntaxParser::ExpressionListContext>(0);
}


size_t syntaxParser::VectorContext::getRuleIndex() const {
  return syntaxParser::RuleVector;
}

void syntaxParser::VectorContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterVector(this);
}

void syntaxParser::VectorContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitVector(this);
}

syntaxParser::VectorContext* syntaxParser::vector() {
  VectorContext *_localctx = _tracker.createInstance<VectorContext>(_ctx, getState());
  enterRule(_localctx, 46, syntaxParser::RuleVector);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(215);
    match(syntaxParser::LBRACKET);
    setState(217);
    _errHandler->sync(this);

    _la = _input->LA(1);
    if ((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 31078387548160) != 0)) {
      setState(216);
      expressionList();
    }
    setState(219);
    match(syntaxParser::RBRACKET);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ArrayAccessContext ------------------------------------------------------------------

syntaxParser::ArrayAccessContext::ArrayAccessContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ArrayAccessContext::IDENTIFIER() {
  return getToken(syntaxParser::IDENTIFIER, 0);
}

tree::TerminalNode* syntaxParser::ArrayAccessContext::LBRACKET() {
  return getToken(syntaxParser::LBRACKET, 0);
}

syntaxParser::ExpressionContext* syntaxParser::ArrayAccessContext::expression() {
  return getRuleContext<syntaxParser::ExpressionContext>(0);
}

tree::TerminalNode* syntaxParser::ArrayAccessContext::RBRACKET() {
  return getToken(syntaxParser::RBRACKET, 0);
}


size_t syntaxParser::ArrayAccessContext::getRuleIndex() const {
  return syntaxParser::RuleArrayAccess;
}

void syntaxParser::ArrayAccessContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterArrayAccess(this);
}

void syntaxParser::ArrayAccessContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitArrayAccess(this);
}

syntaxParser::ArrayAccessContext* syntaxParser::arrayAccess() {
  ArrayAccessContext *_localctx = _tracker.createInstance<ArrayAccessContext>(_ctx, getState());
  enterRule(_localctx, 48, syntaxParser::RuleArrayAccess);

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(221);
    match(syntaxParser::IDENTIFIER);
    setState(222);
    match(syntaxParser::LBRACKET);
    setState(223);
    expression();
    setState(224);
    match(syntaxParser::RBRACKET);
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- ComparisonOpContext ------------------------------------------------------------------

syntaxParser::ComparisonOpContext::ComparisonOpContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::EQ() {
  return getToken(syntaxParser::EQ, 0);
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::NEQ() {
  return getToken(syntaxParser::NEQ, 0);
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::GT() {
  return getToken(syntaxParser::GT, 0);
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::LT() {
  return getToken(syntaxParser::LT, 0);
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::GTE() {
  return getToken(syntaxParser::GTE, 0);
}

tree::TerminalNode* syntaxParser::ComparisonOpContext::LTE() {
  return getToken(syntaxParser::LTE, 0);
}


size_t syntaxParser::ComparisonOpContext::getRuleIndex() const {
  return syntaxParser::RuleComparisonOp;
}

void syntaxParser::ComparisonOpContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterComparisonOp(this);
}

void syntaxParser::ComparisonOpContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitComparisonOp(this);
}

syntaxParser::ComparisonOpContext* syntaxParser::comparisonOp() {
  ComparisonOpContext *_localctx = _tracker.createInstance<ComparisonOpContext>(_ctx, getState());
  enterRule(_localctx, 50, syntaxParser::RuleComparisonOp);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(226);
    _la = _input->LA(1);
    if (!((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 202309632) != 0))) {
    _errHandler->recoverInline(this);
    }
    else {
      _errHandler->reportMatch(this);
      consume();
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

//----------------- LogicalOpContext ------------------------------------------------------------------

syntaxParser::LogicalOpContext::LogicalOpContext(ParserRuleContext *parent, size_t invokingState)
  : ParserRuleContext(parent, invokingState) {
}

tree::TerminalNode* syntaxParser::LogicalOpContext::AND() {
  return getToken(syntaxParser::AND, 0);
}

tree::TerminalNode* syntaxParser::LogicalOpContext::OR() {
  return getToken(syntaxParser::OR, 0);
}

tree::TerminalNode* syntaxParser::LogicalOpContext::XOR() {
  return getToken(syntaxParser::XOR, 0);
}


size_t syntaxParser::LogicalOpContext::getRuleIndex() const {
  return syntaxParser::RuleLogicalOp;
}

void syntaxParser::LogicalOpContext::enterRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->enterLogicalOp(this);
}

void syntaxParser::LogicalOpContext::exitRule(tree::ParseTreeListener *listener) {
  auto parserListener = dynamic_cast<syntaxListener *>(listener);
  if (parserListener != nullptr)
    parserListener->exitLogicalOp(this);
}

syntaxParser::LogicalOpContext* syntaxParser::logicalOp() {
  LogicalOpContext *_localctx = _tracker.createInstance<LogicalOpContext>(_ctx, getState());
  enterRule(_localctx, 52, syntaxParser::RuleLogicalOp);
  size_t _la = 0;

#if __cplusplus > 201703L
  auto onExit = finally([=, this] {
#else
  auto onExit = finally([=] {
#endif
    exitRule();
  });
  try {
    enterOuterAlt(_localctx, 1);
    setState(228);
    _la = _input->LA(1);
    if (!((((_la & ~ 0x3fULL) == 0) &&
      ((1ULL << _la) & 1879048192) != 0))) {
    _errHandler->recoverInline(this);
    }
    else {
      _errHandler->reportMatch(this);
      consume();
    }
   
  }
  catch (RecognitionException &e) {
    _errHandler->reportError(this, e);
    _localctx->exception = std::current_exception();
    _errHandler->recover(this, _localctx->exception);
  }

  return _localctx;
}

void syntaxParser::initialize() {
#if ANTLR4_USE_THREAD_LOCAL_CACHE
  syntaxParserInitialize();
#else
  ::antlr4::internal::call_once(syntaxParserOnceFlag, syntaxParserInitialize);
#endif
}
