package model.exp;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.type.IntType;
import model.value.IValue;
import model.value.IntValue;
import utils.IDict;

public class ArithExp implements IExp {
  private IExp exp1;
  private IExp exp2;
  private char operation;

  public ArithExp(char operation, IExp exp1, IExp exp2) {
    this.exp1 = exp1;
    this.exp2 = exp2;
    this.operation = operation;
  }

  @Override
  public IValue eval(IDict<String, IValue> symTable) throws MyException, ExpressionException {
    IValue v1, v2;
    v1 = exp1.eval(symTable);
    if (v1.getType().equals(new IntType())) {
      v2 = exp2.eval(symTable);
      if (v2.getType().equals(new IntType())) {
        IntValue i1 = (IntValue) v1;
        IntValue i2 = (IntValue) v2;
        int n1, n2;
        n1 = i1.getVal();
        n2 = i2.getVal();
        switch (operation) {
          case '+':
            return new IntValue(n1 + n2);
          case '-':
            return new IntValue(n1 - n2);
          case '*':
            return new IntValue(n1 * n2);
          case '/':
            if (n2 == 0)
              throw new ExpressionException("Division by zero");
            return new IntValue(n1 / n2);
          default:
            throw new MyException("Invalid arithmetic operator");
        }
      } else
        throw new MyException("Operand2 is not an integer");
    } else
      throw new MyException("Operand1 is not an integer");
  }

  @Override
  public IExp deepCopy() {
    return new ArithExp(operation, exp1.deepCopy(), exp2.deepCopy());
  }

  @Override
  public String toString() {
    return exp1.toString() + " " + operation + " " + exp2.toString();
  }
}