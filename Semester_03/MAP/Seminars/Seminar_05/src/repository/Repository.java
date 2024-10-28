package repository;

import java.util.List;
import java.util.ArrayList;
import model.PrgState;

public class Repository implements IRepository {
  private List<PrgState> prgList;

  public Repository(PrgState prg) {
    prgList = new ArrayList<>();
    prgList.add(prg);
  }

  @Override
  public void addPrg(PrgState prg) {
    prgList.add(prg);
  }

  @Override
  public PrgState getCurrentPrg() {
    return prgList.get(0);
  }
}
