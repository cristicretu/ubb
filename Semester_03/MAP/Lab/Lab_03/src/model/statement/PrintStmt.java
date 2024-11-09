package model.statement;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.value.IValue;

public class PrintStmt implements IStmt {
  private IExp exp;

  public PrintStmt(IExp exp) {
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue val;
    try {
      val = this.exp.eval(prg.getSymTable());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    prg.getOutput().add(val);
    return prg;
  }

  @Override
  public String toString() {
    return "print(" + this.exp.toString() + ")";
  }

  @Override
  public IStmt deepCopy() {
    return new PrintStmt(this.exp.deepCopy());
  }
}
