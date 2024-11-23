package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.value.IValue;
import utils.IDict;

public interface IExp {
  IValue eval(IDict<String, IValue> symTable, IDict<Integer, IValue> heap)
      throws MyException, ExpressionException;

  IExp deepCopy();
}
