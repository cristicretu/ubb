package model.statement;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileNotFoundException;

import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.IType;
import model.type.StringType;
import model.value.IValue;
import model.value.StringValue;
import utils.IDict;

public class OpenRFile implements IStmt {
  private IExp exp;

  public OpenRFile(IExp exp) {
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
    if (prg.getFileTable().isDefined(stringValue)) {
      throw new MyException("File is already open");
    }
    try {
      BufferedReader br = new BufferedReader(new FileReader(stringValue.getValue()));
      prg.getFileTable().put(stringValue, br);
    } catch (FileNotFoundException e) {
      throw new MyException("File not found: " + e.getMessage());
    }
    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new OpenRFile(exp.deepCopy());
  }

  @Override
  public String toString() {
    return "openRFile(" + exp.toString() + ")";
  }

  @Override
  public IDict<String, IType> typecheck(IDict<String, IType> typeEnv) throws MyException {
    IType type;
    try {
      type = exp.typecheck(typeEnv);
    } catch (ExpressionException e) {
      throw new MyException(e.getMessage());
    }
    if (!type.equals(new StringType())) {
      throw new MyException("Expression is not a string");
    }
    return typeEnv;
  }
}
