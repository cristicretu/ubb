package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.type.BoolType;
import model.type.IType;
import model.value.BoolValue;
import model.value.IValue;
import utils.IDict;
import utils.IHeap;

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
  public IValue eval(IDict<String, IValue> symTable, IHeap<Integer, IValue> heap)
      throws MyException {
    IValue v1, v2;
    try {
      v1 = exp1.eval(symTable, heap);
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    } catch (MyException e) {
      throw new MyException(e.getMessage());
    }
    if (v1.getType().equals(new BoolType())) {
      try {
        v2 = exp2.eval(symTable, heap);
      } catch (ExpressionException e) {
        throw new MyException(e.getMessage());
      } catch (MyException e) {
        throw new MyException(e.getMessage());
      }
      if (v2.getType().equals(new BoolType())) {
        BoolValue b1 = (BoolValue) v1;
        BoolValue b2 = (BoolValue) v2;
        boolean n1, n2;
        n1 = Boolean.parseBoolean(b1.toString());
        n2 = Boolean.parseBoolean(b2.toString());
        if (operation.equals("&&")) {
          return new BoolValue(n1 && n2);
        } else if (operation.equals("||")) {
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

  @Override
  public IType typecheck(IDict<String, IType> typeEnv) throws ExpressionException {
    IType type1, type2;
    type1 = exp1.typecheck(typeEnv);
    type2 = exp2.typecheck(typeEnv);
    if (type1.equals(new BoolType()) && type2.equals(new BoolType())) {
      return new BoolType();
    } else {
      throw new ExpressionException("Logical expression: operands are not booleans");
    }
  }
}
