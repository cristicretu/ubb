package model.statement;

import controller.MyException;
import model.PrgState;
import model.exp.IExp;
import model.exp.ValueExp;
import model.value.Value;

public class PrintStmt implements IStmt {
  private IExp exp;

  public PrintStmt(ValueExp valueExp) {
    this.exp = valueExp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    Value val = this.exp.eval(prg.getSymTable());
    prg.getOutput().add(val);
    return prg;
  }

  @Override
  public String toString() {
    return "print(" + this.exp.toString() + ")";
  }
}
