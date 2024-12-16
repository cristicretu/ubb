package model.statement;

import exceptions.MyException;
import model.PrgState;
import model.type.IType;
import utils.IDict;

public interface IStmt {
  PrgState execute(PrgState prg) throws MyException;

  IStmt deepCopy();

  IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException;
}
