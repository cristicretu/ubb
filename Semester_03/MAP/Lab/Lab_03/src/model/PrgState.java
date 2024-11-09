package model;

import utils.IStack;
import utils.IDict;
import utils.IList;
import model.statement.IStmt;
import model.value.IValue;

public class PrgState {
  private IStack<IStmt> exeStack;

  public IStack<IStmt> getExeStack() {
    return exeStack;
  }

  private IDict<String, IValue> symTable;

  public IDict<String, IValue> getSymTable() {
    return symTable;
  }

  private IList<IValue> output;

  public IList<IValue> getOutput() {
    return output;
  }

  private IStmt originalProgram;

  // file table
  private IDict<Integer, String> fileTable;

  public IDict<Integer, String> getFileTable() {
    return fileTable;
  }

  public PrgState(IStack<IStmt> exeStack, IDict<String, IValue> symTable, IList<IValue> output, IStmt originalProgram,
      IDict<Integer, String> fileTable) {
    this.exeStack = exeStack;
    this.symTable = symTable;
    this.output = output;
    this.originalProgram = originalProgram.deepCopy();
    this.fileTable = fileTable;

    exeStack.push(originalProgram);
  }

  @Override
  public String toString() {
    return "PrgState{\n" + "exeStack=" + exeStack.getList() + ",\n symTable=" + symTable + ",\n output=" + output
        + ",\n originalProgram="
        + originalProgram + ",\n fileTable=" + fileTable + "\n}";
  }
}
