package model.value;

import model.type.IType;

public interface IValue {
  IType getType();

  IValue deepCopy();

  boolean equals(IValue other);

  // Object getVal();
}
