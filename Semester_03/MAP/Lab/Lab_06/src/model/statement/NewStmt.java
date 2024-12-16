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

public class NewStmt implements IStmt {
  private String varName;
  private IExp expression;

  public NewStmt(String varName, IExp expression) {
    this.varName = varName;
    this.expression = expression;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    if (!prg.getSymTable().isDefined(varName)) {
      throw new MyException("Variable " + varName + " not declared");
    }
    IType type;
    try {
      type = prg.getSymTable().get(varName).getType();
    } catch (DictionaryException e) {
      throw new MyException(e.getMessage());
    }
    if (!(type instanceof RefType)) {
      throw new MyException("Variable " + varName + " is not of type RefType");
    }

    IValue value;
    try {
      value = expression.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (!value.getType().equals(((RefType) type).getInner())) {
      throw new MyException("Type mismatch: expected " + ((RefType) type).getInner() +
          " but got " + value.getType());
    }
    Integer newAddress = prg.getHeap().allocate();
    prg.getHeap().put(newAddress, value);
    prg.getSymTable().put(varName, new RefValue(newAddress, value.getType()));
    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new NewStmt(varName, expression.deepCopy());
  }

  @Override
  public String toString() {
    return "NewStmt(" + varName + ", " + expression + ")";
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    try {
      IType typevar = typeEnv.get(varName);
      IType typexp = expression.typecheck(typeEnv);
      if (typevar.equals(new RefType(typexp))) {
        return typeEnv;
      } else {
        throw new MyException("NEW stmt: right hand side and left hand side have different types");
      }
    } catch (DictionaryException e) {
      throw new MyException("Variable " + varName + " is not defined in type environment");
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
  }
}
