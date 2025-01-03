package model.statement;

import exceptions.MyException;
import model.PrgState;
import model.type.IType;
import utils.IDict;

public class NoOPStmt implements IStmt {
  @Override
  public PrgState execute(PrgState prg) throws MyException {
    return null;
  }

  @Override
  public String toString() {
    return "NOP";
  }

  @Override
  public IStmt deepCopy() {
    return new NoOPStmt();
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    return typeEnv;
  }
}
