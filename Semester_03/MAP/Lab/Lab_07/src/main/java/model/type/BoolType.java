package model.type;

import model.value.BoolValue;
import model.value.IValue;

public class BoolType implements IType {
  private String type = "bool";

  @Override
  public boolean equals(Object other) {
    return other instanceof BoolType;
  }

  @Override
  public IType deepCopy() {
    return new BoolType();
  }

  @Override
  public String toString() {
    return this.type;
  }

  @Override
  public IValue defaultValue() {
    return new BoolValue(false);
  }
}
