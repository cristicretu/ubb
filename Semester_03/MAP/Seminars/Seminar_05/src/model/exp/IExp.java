package model.exp;

import controller.MyException;
import model.value.IValue;
import utils.IDict;

public interface IExp {
  IValue eval(IDict<String, IValue> symTable) throws MyException;

  IExp deepCopy();
}
