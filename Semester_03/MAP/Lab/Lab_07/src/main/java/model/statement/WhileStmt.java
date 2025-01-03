package model.statement;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.BoolType;
import model.type.IType;
import model.value.BoolValue;
import model.value.IValue;
import utils.IDict;

public class WhileStmt implements IStmt {
  private IExp expression;
  private IStmt statement;

  public WhileStmt(IExp expression, IStmt statement) {
    this.expression = expression;
    this.statement = statement;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue value;
    try {
      value = expression.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (!value.getType().equals(new BoolType())) {
      throw new MyException("Expression is not of BoolType");
    }
    BoolValue boolValue = (BoolValue) value;
    if (boolValue.getVal()) {
      prg.getExeStack().push(this);
      prg.getExeStack().push(statement);
    }
    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new WhileStmt(expression.deepCopy(), statement.deepCopy());
  }

  @Override
  public String toString() {
    return "while(" + expression.toString() + ") { " + statement.toString() + " }";
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    try {
      IType typexp = expression.typecheck(typeEnv);
      if (typexp.equals(new BoolType())) {
        statement.typecheck(typeEnv.deepCopy());
        return typeEnv;
      } else {
        throw new MyException("The condition of WHILE has not the type bool");
      }
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
  }
}
