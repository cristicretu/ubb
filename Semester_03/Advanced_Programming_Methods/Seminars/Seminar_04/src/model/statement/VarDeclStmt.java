package model.statement;

import controller.MyException;
import model.PrgState;
import model.type.IType;
import utils.IDict;
import model.Value;

public class VarDeclStmt implements IStmt {
  private String id;
  private IType exp;

  public VarDeclStmt(String id, IType exp) {
    this.id = id;
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IDict<String, Value> symTable = prg.getSymTable();
    if (symTable.isDefined(id)) {
      throw new MyException("Variable " + id + " already declared");
    }

    return null;
  }
}
