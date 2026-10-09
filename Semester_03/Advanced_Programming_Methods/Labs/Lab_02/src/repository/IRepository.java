package repository;

import model.PrgState;

public interface IRepository {
  void addPrg(PrgState prg);

  PrgState getCurrentPrg();
}
