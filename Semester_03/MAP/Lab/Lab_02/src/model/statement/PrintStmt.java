package model.statement;

import controller.MyException;
import model.PrgState;
import model.exp.IExp;
import model.value.IValue;

public class PrintStmt implements IStmt {
  private IExp exp;

  public PrintStmt(IExp exp) { // Changed from ValueExp to IExp
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue val = this.exp.eval(prg.getSymTable());
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
