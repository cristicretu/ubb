package model.statement;

import controller.MyException;
import model.PrgState;
import utils.IStack;

public class CompStmt implements IStmt {
  private IStmt first;
  private IStmt second;

  public CompStmt(IStmt first, IStmt second) {
    this.first = first;
    this.second = second;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IStack<IStmt> stk = prg.getExeStack();

    stk.push(second);
    stk.push(first);

    return prg;
  }

  @Override
  public String toString() {
    return "(" + first.toString() + ";" + second.toString() + ")";
  }

  @Override
  public IStmt deepCopy() {
    return new CompStmt(this.first.deepCopy(), this.second.deepCopy());
  }
}
