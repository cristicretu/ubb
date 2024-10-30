package model.statement;

import exceptions.MyException;
import model.PrgState;

public interface IStmt {
  PrgState execute(PrgState prg) throws MyException;

  IStmt deepCopy();
}
