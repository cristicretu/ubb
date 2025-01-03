package model.statement;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.IType;
import model.value.IValue;
import utils.IDict;

public class PrintStmt implements IStmt {
  private IExp exp;

  public PrintStmt(IExp exp) {
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue val;
    try {
      val = this.exp.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    prg.getOutput().add(val);
    return null;
  }

  @Override
  public String toString() {
    return "print(" + this.exp.toString() + ")";
  }

  @Override
  public IStmt deepCopy() {
    return new PrintStmt(this.exp.deepCopy());
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    try {
      exp.typecheck(typeEnv);
      return typeEnv;
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
  }
}
