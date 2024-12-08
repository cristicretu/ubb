package repository;

import exceptions.MyException;
import model.PrgState;

public interface IRepository {
  void addPrg(PrgState prg);

  PrgState getCurrentPrg();

  void logPrgStateExec(PrgState prg) throws MyException;
}
