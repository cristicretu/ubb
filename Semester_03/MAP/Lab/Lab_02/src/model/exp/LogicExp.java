package model.exp;

import controller.MyException;
import model.type.BoolType;
import model.value.BoolValue;
import model.value.IValue;
import utils.IDict;

public class LogicExp implements IExp {
  private IExp exp1;
  private IExp exp2;
  private String operation;

  public LogicExp(String operation, IExp exp1, IExp exp2) {
    this.exp1 = exp1;
    this.exp2 = exp2;
    this.operation = operation;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable) throws MyException {
    IValue v1, v2;
    v1 = exp1.eval(symTable);
    if (v1.getType().equals(new BoolType())) {
      v2 = exp2.eval(symTable);
      if (v2.getType().equals(new BoolType())) {
        BoolValue b1 = (BoolValue) v1;
        BoolValue b2 = (BoolValue) v2;
        boolean n1, n2;
        n1 = Boolean.parseBoolean(b1.toString());
        n2 = Boolean.parseBoolean(b2.toString());
        if (operation.equals("and")) {
          return new BoolValue(n1 && n2);
        } else if (operation.equals("or")) {
          return new BoolValue(n1 || n2);
        } else {
          throw new MyException("Invalid logical operator");
        }
      } else {
        throw new MyException("Operand2 is not a boolean");
      }
    } else {
      throw new MyException("Operand1 is not a boolean");
    }
  }

  @Override
  public IExp deepCopy() {
    return new LogicExp(operation, exp1.deepCopy(), exp2.deepCopy());
  }

  @Override
  public String toString() {
    return exp1.toString() + " " + operation + " " + exp2.toString();
  }
}