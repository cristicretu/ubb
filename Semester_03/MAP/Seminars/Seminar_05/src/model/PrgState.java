package model;

import utils.IStack;
import utils.IDict;
import utils.IList;
import model.statement.IStmt;
import model.value.Value;

public class PrgState {
  private IStack<IStmt> exeStack;

  public IStack<IStmt> getExeStack() {
    return exeStack;
  }

  private IDict<String, Value> symTable;

  public IDict<String, Value> getSymTable() {
    return symTable;
  }

  private IList<Value> output;

  public IList<Value> getOutput() {
    return output;
  }

  private IStmt originalProgram;

  public PrgState(IStack<IStmt> exeStack, IDict<String, Value> symTable, IList<Value> output, IStmt originalProgram) {
    this.exeStack = exeStack;
    this.symTable = symTable;
    this.output = output;
    this.originalProgram = originalProgram;

    exeStack.push(originalProgram);
  }

  @Override
  public String toString() {
    return "PrgState{\n" + "exeStack=" + exeStack + ",\n symTable=" + symTable + ",\n output=" + output
        + ",\n originalProgram="
        + originalProgram + "\n}";
  }
}
