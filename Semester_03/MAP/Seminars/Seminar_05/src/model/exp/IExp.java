package model.exp;

import controller.MyException;
import model.value.Value;
import utils.IDict;

public interface IExp {
  Value eval(IDict<String, Value> symTable) throws MyException;
}
