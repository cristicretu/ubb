package model.statement;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.BoolType;
import model.value.BoolValue;
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
    IValue val;
    try {
      val = this.exp.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (val.getType().equals(new BoolType())) {
      if (((BoolValue) val).getVal()) {
        prg.getExeStack().push(this.thenStmt);
      } else {
        prg.getExeStack().push(this.elseStmt);
      }
    } else {
      throw new MyException("The condition in the if statement is not a boolean");
    }
    return null;
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
