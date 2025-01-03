package model.statement;

import exceptions.DictionaryException;
import exceptions.ExpressionException;
import exceptions.MyException;
import model.PrgState;
import model.exp.IExp;
import model.type.IType;
import model.type.IntType;
import model.type.StringType;
import model.value.IValue;
import model.value.IntValue;
import model.value.StringValue;
import utils.IDict;

import java.io.BufferedReader;
import java.io.IOException;

public class ReadFile implements IStmt {
  private IExp exp;
  private String varName;

  public ReadFile(IExp exp, String varName) {
    this.exp = exp;
    this.varName = varName;
  }

  @Override
  public PrgState execute(PrgState prg) throws MyException {
    if (!prg.getSymTable().isDefined(varName)) {
      throw new MyException("Variable " + varName + " is not defined");
    }

    IValue varValue;
    try {
      varValue = prg.getSymTable().get(varName);
    } catch (DictionaryException e) {
      throw new MyException(e.getMessage());
    }
    if (!varValue.getType().equals(new IntType())) {
      throw new MyException("Variable " + varName + " is not of type int");
    }

    IValue fileNameValue;
    try {
      fileNameValue = exp.eval(prg.getSymTable(), prg.getHeap());
    } catch (ExpressionException | MyException e) {
      throw new MyException(e.getMessage());
    }
    if (!fileNameValue.getType().equals(new StringType())) {
      throw new MyException("Expression does not evaluate to a string");
    }

    StringValue fileName = (StringValue) fileNameValue;
    BufferedReader br;
    try {
      br = prg.getFileTable().get(fileName);
    } catch (DictionaryException e) {
      throw new MyException(e.getMessage());
    }
    if (br == null) {
      throw new MyException("File " + fileName.getValue() + " is not opened");
    }

    try {
      String line = br.readLine();
      IntValue val;
      if (line == null) {
        val = new IntValue(0);
      } else {
        try {
          val = new IntValue(Integer.parseInt(line));
        } catch (NumberFormatException e) {
          throw new MyException("Invalid integer format in file");
        }
      }
      try {
        prg.getSymTable().update(varName, val);
      } catch (DictionaryException e) {
        throw new MyException(e.getMessage());
      }
    } catch (IOException e) {
      throw new MyException("Error reading from file: " + e.getMessage());
    }

    return null;
  }

  @Override
  public IStmt deepCopy() {
    return new ReadFile(exp.deepCopy(), varName);
  }

  @Override
  public String toString() {
    return "readFile(" + exp.toString() + ", " + varName + ")";
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
