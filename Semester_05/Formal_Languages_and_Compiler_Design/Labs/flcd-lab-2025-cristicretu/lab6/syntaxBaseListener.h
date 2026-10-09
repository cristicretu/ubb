
// Generated from syntax.g4 by ANTLR 4.13.2

#pragma once


#include "antlr4-runtime.h"
#include "syntaxListener.h"


/**
 * This class provides an empty implementation of syntaxListener,
 * which can be extended to create a listener which only needs to handle a subset
 * of the available methods.
 */
class  syntaxBaseListener : public syntaxListener {
public:

  virtual void enterProgram(syntaxParser::ProgramContext * /*ctx*/) override { }
  virtual void exitProgram(syntaxParser::ProgramContext * /*ctx*/) override { }

  virtual void enterStatementList(syntaxParser::StatementListContext * /*ctx*/) override { }
  virtual void exitStatementList(syntaxParser::StatementListContext * /*ctx*/) override { }

  virtual void enterStatement(syntaxParser::StatementContext * /*ctx*/) override { }
  virtual void exitStatement(syntaxParser::StatementContext * /*ctx*/) override { }

  virtual void enterDeclStmt(syntaxParser::DeclStmtContext * /*ctx*/) override { }
  virtual void exitDeclStmt(syntaxParser::DeclStmtContext * /*ctx*/) override { }

  virtual void enterInputStmt(syntaxParser::InputStmtContext * /*ctx*/) override { }
  virtual void exitInputStmt(syntaxParser::InputStmtContext * /*ctx*/) override { }

  virtual void enterOutputStmt(syntaxParser::OutputStmtContext * /*ctx*/) override { }
  virtual void exitOutputStmt(syntaxParser::OutputStmtContext * /*ctx*/) override { }

  virtual void enterIfStmt(syntaxParser::IfStmtContext * /*ctx*/) override { }
  virtual void exitIfStmt(syntaxParser::IfStmtContext * /*ctx*/) override { }

  virtual void enterElseBlock(syntaxParser::ElseBlockContext * /*ctx*/) override { }
  virtual void exitElseBlock(syntaxParser::ElseBlockContext * /*ctx*/) override { }

  virtual void enterForStmt(syntaxParser::ForStmtContext * /*ctx*/) override { }
  virtual void exitForStmt(syntaxParser::ForStmtContext * /*ctx*/) override { }

  virtual void enterFuncDefStmt(syntaxParser::FuncDefStmtContext * /*ctx*/) override { }
  virtual void exitFuncDefStmt(syntaxParser::FuncDefStmtContext * /*ctx*/) override { }

  virtual void enterReturnStmt(syntaxParser::ReturnStmtContext * /*ctx*/) override { }
  virtual void exitReturnStmt(syntaxParser::ReturnStmtContext * /*ctx*/) override { }

  virtual void enterExprStmt(syntaxParser::ExprStmtContext * /*ctx*/) override { }
  virtual void exitExprStmt(syntaxParser::ExprStmtContext * /*ctx*/) override { }

  virtual void enterBlock(syntaxParser::BlockContext * /*ctx*/) override { }
  virtual void exitBlock(syntaxParser::BlockContext * /*ctx*/) override { }

  virtual void enterParameterList(syntaxParser::ParameterListContext * /*ctx*/) override { }
  virtual void exitParameterList(syntaxParser::ParameterListContext * /*ctx*/) override { }

  virtual void enterExpressionList(syntaxParser::ExpressionListContext * /*ctx*/) override { }
  virtual void exitExpressionList(syntaxParser::ExpressionListContext * /*ctx*/) override { }

  virtual void enterCondition(syntaxParser::ConditionContext * /*ctx*/) override { }
  virtual void exitCondition(syntaxParser::ConditionContext * /*ctx*/) override { }

  virtual void enterComparison(syntaxParser::ComparisonContext * /*ctx*/) override { }
  virtual void exitComparison(syntaxParser::ComparisonContext * /*ctx*/) override { }

  virtual void enterExpression(syntaxParser::ExpressionContext * /*ctx*/) override { }
  virtual void exitExpression(syntaxParser::ExpressionContext * /*ctx*/) override { }

  virtual void enterTerm(syntaxParser::TermContext * /*ctx*/) override { }
  virtual void exitTerm(syntaxParser::TermContext * /*ctx*/) override { }

  virtual void enterFactor(syntaxParser::FactorContext * /*ctx*/) override { }
  virtual void exitFactor(syntaxParser::FactorContext * /*ctx*/) override { }

  virtual void enterPostfixOp(syntaxParser::PostfixOpContext * /*ctx*/) override { }
  virtual void exitPostfixOp(syntaxParser::PostfixOpContext * /*ctx*/) override { }

  virtual void enterPrimary(syntaxParser::PrimaryContext * /*ctx*/) override { }
  virtual void exitPrimary(syntaxParser::PrimaryContext * /*ctx*/) override { }

  virtual void enterFunctionCall(syntaxParser::FunctionCallContext * /*ctx*/) override { }
  virtual void exitFunctionCall(syntaxParser::FunctionCallContext * /*ctx*/) override { }

  virtual void enterVector(syntaxParser::VectorContext * /*ctx*/) override { }
  virtual void exitVector(syntaxParser::VectorContext * /*ctx*/) override { }

  virtual void enterArrayAccess(syntaxParser::ArrayAccessContext * /*ctx*/) override { }
  virtual void exitArrayAccess(syntaxParser::ArrayAccessContext * /*ctx*/) override { }

  virtual void enterComparisonOp(syntaxParser::ComparisonOpContext * /*ctx*/) override { }
  virtual void exitComparisonOp(syntaxParser::ComparisonOpContext * /*ctx*/) override { }

  virtual void enterLogicalOp(syntaxParser::LogicalOpContext * /*ctx*/) override { }
  virtual void exitLogicalOp(syntaxParser::LogicalOpContext * /*ctx*/) override { }


  virtual void enterEveryRule(antlr4::ParserRuleContext * /*ctx*/) override { }
  virtual void exitEveryRule(antlr4::ParserRuleContext * /*ctx*/) override { }
  virtual void visitTerminal(antlr4::tree::TerminalNode * /*node*/) override { }
  virtual void visitErrorNode(antlr4::tree::ErrorNode * /*node*/) override { }

};

