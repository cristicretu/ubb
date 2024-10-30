package model.type;

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
}
