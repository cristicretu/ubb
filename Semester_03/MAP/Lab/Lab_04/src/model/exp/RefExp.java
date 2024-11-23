package model.exp;

import exceptions.ExpressionException;
import exceptions.HeapException;
import exceptions.MyException;
import model.value.IValue;
import model.value.RefValue;
import utils.IDict;
import utils.IHeap;

public class RefExp implements IExp {
  private IExp expression;

  public RefExp(IExp expression) {
    this.expression = expression;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException, ExpressionException {
    IValue value = expression.eval(symTable, heap);
    if (!(value instanceof RefValue)) {
      throw new ExpressionException("Expected a RefValue, got " + value);
    }

    RefValue refValue = (RefValue) value;
    Integer address = refValue.getAddress();
    if (!heap.isDefined(address)) {
      throw new ExpressionException("Address " + address + " is not defined in heap");
    }
    try {
      return heap.get(address);
    } catch (HeapException e) {
      throw new MyException(e.getMessage());
    }
  }

  @Override
  public IExp deepCopy() {
    return new RefExp(expression.deepCopy());
  }

  @Override
  public String toString() {
    return "RefExp(" + expression + ")";
  }

}
