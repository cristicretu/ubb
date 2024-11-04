package controller;

import repository.IRepository;
import utils.IStack;
import exceptions.MyException;
import exceptions.StackException;
import model.PrgState;
import model.statement.IStmt;

public class Controller {
  private IRepository repo;

  public Controller(IRepository repo) {
    this.repo = repo;
  }

  public PrgState oneStep(PrgState prg) throws MyException {
    IStack<IStmt> stk = prg.getExeStack();
    if (stk.isEmpty()) {
      throw new MyException("Empty execution stack");
    }

    IStmt currentStmt;
    try {
      currentStmt = stk.pop();
    } catch (StackException e) {
      throw new MyException(e.getMessage());
    }
    return currentStmt.execute(prg);
  }

  public void allSteps() throws MyException {
    PrgState currentPrg = repo.getCurrentPrg();
    System.out.println(currentPrg);
    while (!currentPrg.getExeStack().isEmpty()) {
      oneStep(currentPrg);
      System.out.println(currentPrg);
    }
  }
}
