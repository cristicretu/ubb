package controller;

import repository.IRepository;
import utils.IHeap;

import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutionException;
import java.util.stream.Collectors;

import exceptions.MyException;
import model.PrgState;
import model.value.IValue;

public class Controller {
  private IRepository repo;
  private ExecutorService executor;

  public Controller(IRepository repo) {
    this.repo = repo;
    this.executor = Executors.newFixedThreadPool(2);
  }

  public List<PrgState> removeCompletedPrg(List<PrgState> inPrgList) {
    return inPrgList.stream()
        .filter(p -> p.isNotCompleted() && !p.getExeStack().isEmpty())
        .collect(Collectors.toList());
  }

  private void oneStepForAllPrg(List<PrgState> prgList) throws InterruptedException {
    prgList.forEach(prg -> {
      try {
        repo.logPrgStateExec(prg);
      } catch (MyException e) {
        System.out.println(e.getMessage());
      }
    });

    List<Callable<PrgState>> callList = prgList.stream()
        .filter(p -> !p.getExeStack().isEmpty())
        .map((PrgState p) -> (Callable<PrgState>) (() -> {
          return p.oneStep();
        }))
        .collect(Collectors.toList());

    if (callList.isEmpty()) {
      prgList.forEach(p -> p.setNotCompleted(false));
      return;
    }

    List<PrgState> newPrgList = executor.invokeAll(callList).stream()
        .map(future -> {
          try {
            return future.get();
          } catch (Exception e) {
            System.out.println(e.getMessage());
            return null;
          }
        })
        .filter(p -> p != null)
        .collect(Collectors.toList());

    prgList.addAll(newPrgList);
  }

  public void allSteps() throws MyException {
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
