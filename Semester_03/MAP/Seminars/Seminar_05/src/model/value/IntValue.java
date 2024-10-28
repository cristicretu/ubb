package model.value;

import model.type.IType;
import model.type.IntType;

public class IntValue implements Value {
  private int val;

  public IntValue(int val) {
    this.val = val;
  }

  @Override
  public IType getType() {
    return new IntType();
  }

  @Override
  public String toString() {
    return String.valueOf(val);
  }
}
