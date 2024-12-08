package controller;

import repository.IRepository;
import utils.IHeap;
import utils.IStack;
import utils.MyHeap;

import java.util.List;

import exceptions.MyException;
import exceptions.StackException;
import model.PrgState;
import model.statement.IStmt;
import model.value.IValue;

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
    List<PrgState> prgList = repo.getPrgList();
    for (PrgState prg : prgList) {
      repo.logPrgStateExec(prg);
      while (!prg.getExeStack().isEmpty()) {
        oneStep(prg);
        repo.logPrgStateExec(prg);

        IHeap<Integer, IValue> heap = prg.getHeap();
        heap.setHeap(heap.safeGarbageCollector(prg.getUsedAddresses(), heap.getHeap()));
      }
    }
  }
}
