package model.exp;

import controller.MyException;
import model.value.IValue;
import utils.IDict;

public class VariableExp implements IExp {
  private IValue value;

  public VariableExp(IValue value) {
    this.value = value;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable) throws MyException {
    return this.value;
  }

  @Override
  public IExp deepCopy() {
    return new VariableExp(this.value.deepCopy());
  }
}
