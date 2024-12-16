package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.type.IntType;
import model.value.BoolValue;
import model.value.IValue;
import model.value.IntValue;
import utils.IDict;
import utils.IHeap;

public class RelExp implements IExp {
  private IExp exp1;
  private IExp exp2;
  private String operation;

  public RelExp(IExp exp1, IExp exp2, String operation) {
    this.exp1 = exp1;
    this.exp2 = exp2;
    this.operation = operation;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException, ExpressionException {
    IValue v1, v2;
    v1 = exp1.eval(symTable, heap);
    v2 = exp2.eval(symTable, heap);

    if (v1.getType().equals(new IntType()) && v2.getType().equals(new IntType())) {
      IntValue i1 = (IntValue) v1;
      IntValue i2 = (IntValue) v2;
      int n1, n2;
      n1 = i1.getVal();
      n2 = i2.getVal();
      switch (operation) {
        case "<":
          return new BoolValue(n1 < n2);
        case "<=":
          return new BoolValue(n1 <= n2);
        case ">":
          return new BoolValue(n1 > n2);
        case ">=":
          return new BoolValue(n1 >= n2);
        case "==":
          return new BoolValue(n1 == n2);
        case "!=":
          return new BoolValue(n1 != n2);
        default:
          throw new MyException("Invalid relational operator");
      }
    }

    throw new MyException("Operands are not of type int");
  }

  @Override
  public IExp deepCopy() {
    return new RelExp(exp1.deepCopy(), exp2.deepCopy(), operation);
  }

}
