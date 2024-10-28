package model.exp;

import controller.MyException;
import model.value.Value;
import utils.IDict;

public class ValueExp implements IExp {
  private String id;

  public ValueExp(String id) {
    this.id = id;
  }

  @Override
  public Value eval(IDict<String, Value> symTable) throws MyException {
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
}
