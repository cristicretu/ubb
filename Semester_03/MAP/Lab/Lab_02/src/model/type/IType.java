package model.type;

public interface IType {
  boolean equals(Object other);

  IType deepCopy();
}
