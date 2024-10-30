package model.statement;

import controller.MyException;
import model.PrgState;
import model.exp.IExp;
import model.value.IValue;

public class IfStmt implements IStmt {
  private IExp exp;
  private IStmt thenStmt;
  private IStmt elseStmt;

  public IfStmt(IExp exp, IStmt thenStmt, IStmt elseStmt) {
    this.exp = exp;
    this.thenStmt = thenStmt;
    this.elseStmt = elseStmt;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue val = this.exp.eval(prg.getSymTable());
    if (val.getType().equals(new BoolType())) {
      if (val.getVal().equals(true)) {
        prg.getExeStack().push(this.thenStmt);
      } else {
        prg.getExeStack().push(this.elseStmt);
      }
    }
  }

  @Override
  public String toString() {
    return "if (" + this.exp.toString() + ") then (" + this.thenStmt.toString() + ") else (" + this.elseStmt.toString()
        + ")";
  }

  @Override
  public IStmt deepCopy() {
    return new IfStmt(this.exp.deepCopy(), this.thenStmt.deepCopy(), this.elseStmt.deepCopy());
  }
}
