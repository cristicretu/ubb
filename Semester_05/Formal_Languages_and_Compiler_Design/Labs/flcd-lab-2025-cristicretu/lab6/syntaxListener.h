
// Generated from syntax.g4 by ANTLR 4.13.2

#pragma once


#include "antlr4-runtime.h"
#include "syntaxParser.h"


/**
 * This interface defines an abstract listener for a parse tree produced by syntaxParser.
 */
class  syntaxListener : public antlr4::tree::ParseTreeListener {
public:

  virtual void enterProgram(syntaxParser::ProgramContext *ctx) = 0;
  virtual void exitProgram(syntaxParser::ProgramContext *ctx) = 0;

  virtual void enterStatementList(syntaxParser::StatementListContext *ctx) = 0;
  virtual void exitStatementList(syntaxParser::StatementListContext *ctx) = 0;

  virtual void enterStatement(syntaxParser::StatementContext *ctx) = 0;
  virtual void exitStatement(syntaxParser::StatementContext *ctx) = 0;

  virtual void enterDeclStmt(syntaxParser::DeclStmtContext *ctx) = 0;
  virtual void exitDeclStmt(syntaxParser::DeclStmtContext *ctx) = 0;

  virtual void enterInputStmt(syntaxParser::InputStmtContext *ctx) = 0;
  virtual void exitInputStmt(syntaxParser::InputStmtContext *ctx) = 0;

  virtual void enterOutputStmt(syntaxParser::OutputStmtContext *ctx) = 0;
  virtual void exitOutputStmt(syntaxParser::OutputStmtContext *ctx) = 0;

  virtual void enterIfStmt(syntaxParser::IfStmtContext *ctx) = 0;
  virtual void exitIfStmt(syntaxParser::IfStmtContext *ctx) = 0;

  virtual void enterElseBlock(syntaxParser::ElseBlockContext *ctx) = 0;
  virtual void exitElseBlock(syntaxParser::ElseBlockContext *ctx) = 0;

  virtual void enterForStmt(syntaxParser::ForStmtContext *ctx) = 0;
  virtual void exitForStmt(syntaxParser::ForStmtContext *ctx) = 0;

  virtual void enterFuncDefStmt(syntaxParser::FuncDefStmtContext *ctx) = 0;
  virtual void exitFuncDefStmt(syntaxParser::FuncDefStmtContext *ctx) = 0;

  virtual void enterReturnStmt(syntaxParser::ReturnStmtContext *ctx) = 0;
  virtual void exitReturnStmt(syntaxParser::ReturnStmtContext *ctx) = 0;

  virtual void enterExprStmt(syntaxParser::ExprStmtContext *ctx) = 0;
  virtual void exitExprStmt(syntaxParser::ExprStmtContext *ctx) = 0;

  virtual void enterBlock(syntaxParser::BlockContext *ctx) = 0;
  virtual void exitBlock(syntaxParser::BlockContext *ctx) = 0;

  virtual void enterParameterList(syntaxParser::ParameterListContext *ctx) = 0;
  virtual void exitParameterList(syntaxParser::ParameterListContext *ctx) = 0;

  virtual void enterExpressionList(syntaxParser::ExpressionListContext *ctx) = 0;
  virtual void exitExpressionList(syntaxParser::ExpressionListContext *ctx) = 0;

  virtual void enterCondition(syntaxParser::ConditionContext *ctx) = 0;
  virtual void exitCondition(syntaxParser::ConditionContext *ctx) = 0;

  virtual void enterComparison(syntaxParser::ComparisonContext *ctx) = 0;
  virtual void exitComparison(syntaxParser::ComparisonContext *ctx) = 0;

  virtual void enterExpression(syntaxParser::ExpressionContext *ctx) = 0;
  virtual void exitExpression(syntaxParser::ExpressionContext *ctx) = 0;

  virtual void enterTerm(syntaxParser::TermContext *ctx) = 0;
  virtual void exitTerm(syntaxParser::TermContext *ctx) = 0;

  virtual void enterFactor(syntaxParser::FactorContext *ctx) = 0;
  virtual void exitFactor(syntaxParser::FactorContext *ctx) = 0;

  virtual void enterPostfixOp(syntaxParser::PostfixOpContext *ctx) = 0;
  virtual void exitPostfixOp(syntaxParser::PostfixOpContext *ctx) = 0;

  virtual void enterPrimary(syntaxParser::PrimaryContext *ctx) = 0;
  virtual void exitPrimary(syntaxParser::PrimaryContext *ctx) = 0;

  virtual void enterFunctionCall(syntaxParser::FunctionCallContext *ctx) = 0;
  virtual void exitFunctionCall(syntaxParser::FunctionCallContext *ctx) = 0;

  virtual void enterVector(syntaxParser::VectorContext *ctx) = 0;
  virtual void exitVector(syntaxParser::VectorContext *ctx) = 0;

  virtual void enterArrayAccess(syntaxParser::ArrayAccessContext *ctx) = 0;
  virtual void exitArrayAccess(syntaxParser::ArrayAccessContext *ctx) = 0;

  virtual void enterComparisonOp(syntaxParser::ComparisonOpContext *ctx) = 0;
  virtual void exitComparisonOp(syntaxParser::ComparisonOpContext *ctx) = 0;

  virtual void enterLogicalOp(syntaxParser::LogicalOpContext *ctx) = 0;
  virtual void exitLogicalOp(syntaxParser::LogicalOpContext *ctx) = 0;


};

