package model.statement;

import controller.MyException;
import model.PrgState;
import model.Value;
import model.exp.IExp;
import utils.IDict;

public class AssignStmt implements IStmt {
  private String id;
  private IExp exp;

  public AssignStmt(String id, IExp exp) {
    this.id = id;
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IDict<String, Value> symTable = prg.getSymTable();
    if (symTable.isDefined(id)) {
      Value val = this.exp.eval(symTable);
      if (val.getType().equals(symTable.get(id).getType())) {
        symTable.put(id, val);
      } else {
        throw new MyException("Declared type of variable " + id + " and type of the assigned expression do not match");
      }
    } else {
      throw new MyException("The used variable " + id + " was not declared before");
    }
    return prg;
  }

}
