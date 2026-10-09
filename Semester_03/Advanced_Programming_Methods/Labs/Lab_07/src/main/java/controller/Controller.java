package controller;

import repository.IRepository;
import utils.IDict;
import utils.IHeap;
import utils.MyDict;

import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutionException;
import java.util.stream.Collectors;

import exceptions.MyException;
import model.PrgState;
import model.type.IType;
import model.value.IValue;

public class Controller {
  private IRepository repo;
  private ExecutorService executor;
  private boolean typeChecked;

  public Controller(IRepository repo) {
    this.repo = repo;
    this.executor = Executors.newFixedThreadPool(2);
    this.typeChecked = false;
  }

  public IRepository getRepo() {
    return repo;
  }

  public List<PrgState> removeCompletedPrg(List<PrgState> inPrgList) {
    return inPrgList.stream()
        .filter(p -> p.isNotCompleted())
        .collect(Collectors.toList());
  }

  public void oneStepForAllPrg(List<PrgState> prgList) throws InterruptedException {
    prgList.forEach(prg -> {
      try {
        repo.logPrgStateExec(prg);
      } catch (MyException e) {
        System.out.println(e.getMessage());
      }
    });

    List<Callable<PrgState>> callList = prgList.stream()
        .map((PrgState p) -> (Callable<PrgState>) (() -> {
          try {
            return p.oneStep();
          } catch (MyException e) {
            p.setNotCompleted(false);
            System.out.println(e.getMessage());
            return null;
          }
        }))
        .collect(Collectors.toList());

    List<PrgState> newPrgList = executor.invokeAll(callList).stream()
        .map(future -> {
          try {
            return future.get();
          } catch (InterruptedException | ExecutionException e) {
            System.out.println(e.getMessage());
            return null;
          }
        })
        .filter(p -> p != null && p.isNotCompleted())
        .collect(Collectors.toList());

    prgList.addAll(newPrgList);

    prgList.forEach(prg -> {
      try {
        repo.logPrgStateExec(prg);
      } catch (MyException e) {
        System.out.println(e.getMessage());
      }
    });

    repo.setPrgList(prgList);
  }

  private void typecheck() throws MyException {
    PrgState prg = repo.getPrgList().get(0);
    IDict<String, IType> typeEnv = new MyDict<>();
    prg.getOriginalProgram().typecheck(typeEnv);
    typeChecked = true;
  }

  public void allSteps() throws MyException {
    if (!typeChecked) {
      typecheck();
    }

    executor = Executors.newFixedThreadPool(2);
    List<PrgState> prgList = removeCompletedPrg(repo.getPrgList());

    while (prgList.size() > 0) {
      IHeap<Integer, IValue> heap = prgList.get(0).getHeap();

      Set<Integer> usedAddresses = prgList.stream()
          .flatMap(p -> p.getUsedAddresses().stream())
          .collect(Collectors.toSet());

      heap.setHeap(heap.safeGarbageCollector(usedAddresses, heap.getHeap()));

      try {
        oneStepForAllPrg(prgList);
      } catch (InterruptedException e) {
        throw new MyException("Program execution interrupted");
      }

      prgList = removeCompletedPrg(repo.getPrgList());
    }

    executor.shutdownNow();
    repo.setPrgList(prgList);
  }
}