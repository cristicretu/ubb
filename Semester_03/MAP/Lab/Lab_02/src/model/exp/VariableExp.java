package model.exp;

import controller.MyException;
import model.value.IValue;
import utils.IDict;

public class VariableExp implements IExp {
  private String id;

  public VariableExp(String id) {
    this.id = id;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable) throws MyException {
    if (symTable.isDefined(this.id)) {
      return symTable.get(this.id);
    } else {
      throw new MyException("The used variable " + this.id + " was not declared before");
    }
  }

  @Override
  public String toString() {
    return this.id;
  }

  @Override
  public IExp deepCopy() {
    return new VariableExp(this.id);
  }
}
