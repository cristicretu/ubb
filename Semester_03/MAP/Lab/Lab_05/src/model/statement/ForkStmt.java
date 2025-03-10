package model.statement;

import exceptions.MyException;
import model.PrgState;
import utils.MyStack;

public class ForkStmt implements IStmt {
  private IStmt statement;

  public ForkStmt(IStmt statement) {
    this.statement = statement;
  }

  @Override
  public PrgState execute(PrgState currentPrg) throws MyException {
    return new PrgState(
        new MyStack<>(),
        currentPrg.getSymTable().deepCopy(),
        currentPrg.getOutput(),
        statement,
        currentPrg.getFileTable(),
        currentPrg.getHeap());
  }

  @Override
  public IStmt deepCopy() {
    return new ForkStmt(statement.deepCopy());
  }

  @Override
  public String toString() {
    return "fork(" + statement.toString() + ")";
  }
}