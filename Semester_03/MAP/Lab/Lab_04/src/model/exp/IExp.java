package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.value.IValue;
import utils.IDict;
import utils.IHeap;

public interface IExp {
  IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException, ExpressionException;

  IExp deepCopy();
}
