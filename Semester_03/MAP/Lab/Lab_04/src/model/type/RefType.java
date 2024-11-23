package model.type;

import model.value.IValue;
import model.value.RefValue;

public class RefType implements IType {
  private IType inner;

  public RefType(IType inner) {
    this.inner = inner;
  }

  public IType getInner() {
    return inner;
  }

  @Override
  public IType deepCopy() {
    return new RefType(inner.deepCopy());
  }

  @Override
  public IValue defaultValue() {
    return new RefValue(0, inner);
  }

  public boolean equals(IType other) {
    return other instanceof RefType && inner.equals(((RefType) other).inner);
  }
}
