package model.type;

import model.value.IValue;
import model.value.IntValue;

public class IntType implements IType {
  String type = "Int";

  @Override
  public boolean equals(Object obj) {
    return obj instanceof IntType;
  }

  @Override
  public String toString() {
    return type;
  }

  @Override
  public IType deepCopy() {
    return new IntType();
  }

  @Override
  public IValue defaultValue() {
    return new IntValue(0);
  }
}
