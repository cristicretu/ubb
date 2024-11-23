package model;

import utils.IStack;
import utils.IDict;
import utils.IList;
import model.statement.IStmt;
import model.value.IValue;
import model.value.StringValue;

import java.io.BufferedReader;

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

  private IDict<StringValue, BufferedReader> fileTable;

  public IDict<StringValue, BufferedReader> getFileTable() {
    return fileTable;
  }

  private IDict<Integer, IValue> heap;

  public IDict<Integer, IValue> getHeap() {
    return heap;
  }

  public PrgState(IStack<IStmt> exeStack, IDict<String, IValue> symTable, IList<IValue> output, IStmt originalProgram,
      IDict<StringValue, BufferedReader> fileTable, IDict<Integer, IValue> heap) {
    this.exeStack = exeStack;
    this.symTable = symTable;
    this.output = output;
    this.originalProgram = originalProgram.deepCopy();
    this.fileTable = fileTable;
    this.heap = heap;
    exeStack.push(originalProgram);
  }

  @Override
  public String toString() {
    return "PrgState{\n" + "exeStack=" + exeStack.getList() + ",\n symTable=" + symTable + ",\n output=" + output
        + ",\n originalProgram="
        + originalProgram + ",\n fileTable=" + fileTable + ",\n heap=" + heap + "\n}";
  }
}
