package model.statement;

import controller.MyException;
import model.PrgState;
import model.type.IType;
import utils.IDict;
import model.Value;

public class VarDeclStmt implements IStmt {
  private String id;
  private IType type;

  public VarDeclStmt(String id, IType type) {
    this.id = id;
    this.type = type;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IDict<String, Value> symTable = prg.getSymTable();
    if (symTable.isDefined(id)) {
      throw new MyException("Variable " + id + " already declared");
    }
    // Initialize variable with default value based on its type
    // if (type.toString().equals("Int")) {
    // symTable.put(id, new IntValue(0));
    // }
    // Add more type initializations as needed

    return prg;
  }

  @Override
  public String toString() {
    return type.toString() + " " + id;
  }
}
