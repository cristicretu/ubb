package model.exp;

import controller.MyException;
import model.value.IValue;
import utils.IDict;

public class ConstantValue implements IExp {
  private IValue value;

  public ConstantValue(IValue value) {
    this.value = value;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable) throws MyException {
    return this.value;
  }

  @Override
  public IExp deepCopy() {
    return new ConstantValue(this.value.deepCopy());
  }

  @Override
  public String toString() {
    return this.value.toString();
  }
}
