package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.type.IType;
import model.value.IValue;
import utils.IDict;
import utils.IHeap;

public class ConstantValue implements IExp {
  private IValue value;

  public ConstantValue(IValue value) {
    this.value = value;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException {
    return this.value;
  }

  @Override
  public IExp deepCopy() {
    return new ConstantValue(this.value.deepCopy());
  }

  @Override
  public String toString() {
    return this.value.toString();
  }

  @Override
  public IType typecheck(IDict<String, IType> typeEnv) throws ExpressionException {
    return this.value.getType();
  }
}
