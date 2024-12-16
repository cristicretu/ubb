package model;

import utils.IStack;
import utils.IDict;
import utils.IHeap;
import utils.IList;
import model.statement.IStmt;
import model.value.IValue;
import model.value.RefValue;
import model.value.StringValue;

import java.io.BufferedReader;
import java.util.HashSet;
import java.util.Set;

import exceptions.MyException;
import exceptions.StackException;

public class PrgState {
  private static int nextId = 0;
  private final int id;
  private boolean isNotCompleted;

  public boolean isNotCompleted() {
    return isNotCompleted;
  }

  public void setNotCompleted(boolean isNotCompleted) {
    this.isNotCompleted = isNotCompleted;
  }

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

  public IStmt getOriginalProgram() {
    return originalProgram;
  }

  private IDict<StringValue, BufferedReader> fileTable;

  public IDict<StringValue, BufferedReader> getFileTable() {
    return fileTable;
  }

  private IHeap<Integer, IValue> heap;

  public IHeap<Integer, IValue> getHeap() {
    return heap;
  }

  private static synchronized int getNextId() {
    return nextId++;
  }

  public PrgState(IStack<IStmt> exeStack, IDict<String, IValue> symTable, IList<IValue> output, IStmt originalProgram,
      IDict<StringValue, BufferedReader> fileTable, IHeap<Integer, IValue> heap) {
    this.id = getNextId();
    this.exeStack = exeStack;
    this.symTable = symTable;
    this.output = output;
    this.originalProgram = originalProgram.deepCopy();
    this.fileTable = fileTable;
    this.heap = heap;
    this.isNotCompleted = true;
    exeStack.push(originalProgram);
  }

  @Override
  public String toString() {
    return "PrgState{\n" + "id=" + id + ",\n exeStack=" + exeStack.getList() + ",\n symTable=" + symTable
        + ",\n output=" + output
        + ",\n originalProgram="
        + originalProgram + ",\n fileTable=" + fileTable + ",\n heap=" + heap + "\n}";
  }

  public Set<Integer> getUsedAddresses() {
    Set<Integer> usedAddresses = new HashSet<>();
    for (IValue value : this.symTable.getValues()) {
      if (value instanceof RefValue) {
        usedAddresses.add(((RefValue) value).getAddress());
      }
    }

    for (IValue value : this.heap.getValues()) {
      if (value instanceof RefValue) {
        usedAddresses.add(((RefValue) value).getAddress());
      }
    }

    return usedAddresses;
  }

  public PrgState oneStep() throws MyException {
    if (exeStack.isEmpty()) {
      this.isNotCompleted = false;
      return null;
    }

    IStmt crtStmt;
    try {
      crtStmt = exeStack.pop();
    } catch (StackException e) {
      throw new MyException("prgstate stack is empty2");
    }
    return crtStmt.execute(this);
  }

  public int getId() {
    return id;
  }
}
