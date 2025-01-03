package model.statement;

import exceptions.DictionaryException;
import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.IType;
import model.type.StringType;
import model.value.IValue;
import model.value.StringValue;
import utils.IDict;

import java.io.BufferedReader;
import java.io.IOException;

public class CloseRFile implements IStmt {
  private IExp exp;

  public CloseRFile(IExp exp) {
    this.exp = exp;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    IValue value;
    try {
      value = exp.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (!value.getType().equals(new StringType())) {
      throw new MyException("Expression is not a string");
    }
    StringValue stringValue = (StringValue) value;

    BufferedReader br;
    try {
      br = prg.getFileTable().get(stringValue);
    } catch (DictionaryException e) {
      throw new MyException(e.getMessage());
    }
    if (br == null) {
      throw new MyException("File " + stringValue.getValue() + " is not opened");
    }

    try {
      br.close();
    } catch (IOException e) {
      throw new MyException("Error closing file: " + e.getMessage());
    }

    prg.getFileTable().put(stringValue, null);

    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new CloseRFile(exp.deepCopy());
  }

  @Override
  public String toString() {
    return "closeRFile(" + exp.toString() + ")";
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    IType typexp;
    try {
      typexp = exp.typecheck(typeEnv);
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
    if (!typexp.equals(new StringType())) {
      throw new MyException("Expression is not a string");
    }
    return typeEnv;
  }
}
