package model.statement;

import exceptions.MyException;
import model.PrgState;
import model.type.IType;
import model.value.IValue;
import utils.IDict;

public class VarDeclStmt implements IStmt {
  private String id;
  private IType type;

  public VarDeclStmt(String id, IType type) {
    this.id = id;
    this.type = type;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IDict<String, IValue> symTable = prg.getSymTable();
    if (symTable.isDefined(id)) {
      throw new MyException("Variable " + id + " already declared");
    }
    symTable.put(id, type.defaultValue());

    return null;
  }

  @Override
  public String toString() {
    return type.toString() + " " + id;
  }

  @Override
  public IStmt deepCopy() {
    return new VarDeclStmt(id, type.deepCopy());
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    typeEnv.put(id, type);
    return typeEnv;
  }
}
