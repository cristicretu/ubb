package model.statement;

import controller.MyException;
import model.PrgState;
import model.exp.IExp;
import model.value.IValue;
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
    IDict<String, IValue> symTable = prg.getSymTable();
    if (symTable.isDefined(id)) {
      IValue val = this.exp.eval(symTable);
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

  @Override
  public String toString() {
    return id + " = " + exp.toString();
  }

  @Override
  public IStmt deepCopy() {
    return new AssignStmt(this.id, this.exp.deepCopy());
  }

}
