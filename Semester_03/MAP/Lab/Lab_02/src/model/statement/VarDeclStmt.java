package model.statement;

import controller.MyException;
import model.PrgState;
import model.type.IType;
import model.type.IntType;
import model.value.IntValue;
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
    if (type.toString().equals("Int")) {
      symTable.put(id, new IntValue(0));
    }

    return prg;
  }

  @Override
  public String toString() {
    return type.toString() + " " + id;
  }

  @Override
  public IStmt deepCopy() {
    return new VarDeclStmt(id, new IntType());
  }
}
