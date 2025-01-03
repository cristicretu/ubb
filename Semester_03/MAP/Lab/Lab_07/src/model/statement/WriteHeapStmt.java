package model.statement;

import exceptions.DictionaryException;
import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.IType;
import model.type.RefType;
import model.value.IValue;
import model.value.RefValue;
import utils.IDict;

public class WriteHeapStmt implements IStmt {
  private String varName;
  private IExp expression;

  public WriteHeapStmt(String varName, IExp expression) {
    this.varName = varName;
    this.expression = expression;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    if (!prg.getSymTable().isDefined(varName)) {
      throw new MyException("Variable " + varName + " not declared");
    }

    IValue varValue;
    try {
      varValue = prg.getSymTable().get(varName);
    } catch (DictionaryException e) {
      throw new MyException(e.getMessage());
    }
    if (!(varValue instanceof RefValue)) {
      throw new MyException("Variable " + varName + " is not of RefType");
    }

    RefValue refValue = (RefValue) varValue;
    Integer address = refValue.getAddress();

    if (!prg.getHeap().isDefined(address)) {
      throw new MyException("Address " + address + " is not allocated in heap");
    }

    IValue value;
    try {
      value = expression.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (!value.getType().equals(((RefType) refValue.getType()).getInner())) {
      throw new MyException("Type of expression and type of variable do not match");
    }

    prg.getHeap().put(address, value);
    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new WriteHeapStmt(varName, expression.deepCopy());
  }

  @Override
  public String toString() {
    return "WriteHeapStmt(" + varName + ", " + expression + ")";
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    try {
      IType typevar = typeEnv.get(varName);
      IType typexp = expression.typecheck(typeEnv);
      if (typevar instanceof RefType) {
        RefType reft = (RefType) typevar;
        if (typexp.equals(reft.getInner())) {
          return typeEnv;
        } else {
          throw new MyException("WriteHeapStmt: right hand side and left hand side have different types");
        }
      } else {
        throw new MyException("WriteHeapStmt: " + varName + " is not of RefType");
      }
    } catch (DictionaryException e) {
      throw new MyException("Variable " + varName + " is not defined in type environment");
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
  }
}
