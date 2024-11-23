package model.exp;

import exceptions.DictionaryException;
import exceptions.MyException;
import model.value.IValue;
import utils.IDict;
import utils.IHeap;

public class VariableExp implements IExp {
  private String id;

  public VariableExp(String id) {
    this.id = id;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException {
    if (symTable.isDefined(this.id)) {
      try {
        return symTable.get(this.id);
      } catch (DictionaryException e) {
        throw new MyException(e.getMessage());
      }
    } else {
      throw new MyException("The used variable " + this.id + " was not declared before");
    }
  }

  @Override
  public String toString() {
    return this.id;
  }

  @Override
  public IExp deepCopy() {
    return new VariableExp(this.id);
  }
}
