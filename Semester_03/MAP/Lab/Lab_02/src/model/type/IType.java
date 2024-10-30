package model.type;

import model.value.IValue;

public interface IType {
  boolean equals(Object other);

  IType deepCopy();

  IValue defaultValue();
}
