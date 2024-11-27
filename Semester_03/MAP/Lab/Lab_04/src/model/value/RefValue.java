package model.value;

import model.type.IType;
import model.type.RefType;

public class RefValue implements IValue {
  private int address;
  private IType locationType;

  public RefValue(int address, IType locationType) {
    this.address = address;
    this.locationType = locationType;
  }

  @Override
  public IType getType() {
    return new RefType(locationType);
  }

  public int getAddress() {
    return address;
  }

  @Override
  public IValue deepCopy() {
    return new RefValue(address, locationType);
  }

  @Override
  public boolean equals(IValue other) {
    return other instanceof RefValue && address == ((RefValue) other).address;
  }

  @Override
  public String toString() {
    return "RefValue(" + address + ", " + locationType.toString() + ")";
  }
}
