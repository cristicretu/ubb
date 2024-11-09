package model.value;

import model.type.IType;
import model.type.StringType;

public class StringValue implements IValue {
  private String value;

  public StringValue(String value) {
    this.value = value;
  }

  @Override
  public IType getType() {
    return new StringType();
  }

  public String getValue() {
    return value;
  }

  @Override
  public IValue deepCopy() {
    return new StringValue(value);
  }

  @Override
  public boolean equals(IValue other) {
    return other instanceof StringValue && ((StringValue) other).value.equals(value);
  }

  @Override
  public String toString() {
    return value;
  }
}
