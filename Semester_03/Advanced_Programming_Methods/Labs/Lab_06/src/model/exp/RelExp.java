package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.type.BoolType;
import model.type.IType;
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

  @Override
  public IType typecheck(IDict<String, IType> typeEnv) throws ExpressionException {
    IType type1, type2;
    type1 = exp1.typecheck(typeEnv);
    type2 = exp2.typecheck(typeEnv);
    if (type1.equals(new IntType()) && type2.equals(new IntType())) {
      return new BoolType();
    } else {
      throw new ExpressionException("RelExp: operands are not integers");
    }
  }

}
