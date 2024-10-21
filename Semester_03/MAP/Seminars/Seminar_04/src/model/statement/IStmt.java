package model;

import controller.MyException;

public interface IStmt {
  PrgState execute(PrgState prg) throws MyException;
}
