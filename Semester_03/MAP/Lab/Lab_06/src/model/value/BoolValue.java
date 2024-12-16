package model.value;

import model.type.BoolType;
import model.type.IType;

public class BoolValue implements IValue {
  private boolean val;

  public BoolValue(boolean val) {
    this.val = val;
  }

  @Override
  public IType getType() {
    return new BoolType();
  }

  @Override
  public String toString() {
    return String.valueOf(val);
  }

  @Override
  public IValue deepCopy() {
    return new BoolValue(this.val);
  }

  public boolean getVal() {
    return this.val;
  }

  @Override
  public boolean equals(IValue other) {
    return other instanceof BoolValue && ((BoolValue) other).val == this.val;
  }
}
