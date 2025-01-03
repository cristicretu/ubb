package repository;

import java.util.List;
import java.util.Scanner;

import exceptions.MyException;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import model.PrgState;

public class Repository implements IRepository {
  private List<PrgState> prgList;
  private String logFilePath;

  public Repository(PrgState prg) {
    prgList = new ArrayList<>();
    prgList.add(prg);
  }

  public Repository(PrgState prg, String logFilePath) {
    this.prgList = new ArrayList<>();
    this.prgList.add(prg);
    this.logFilePath = logFilePath;
  }

  public void setLogFilePath() {
    Scanner scanner = new Scanner(System.in);
    System.out.println("Enter the log file path: ");
    this.logFilePath = scanner.nextLine();
    scanner.close();
  }

  @Override
  public void addPrg(PrgState prg) {
    prgList.add(prg);
  }

  @Override
  public void logPrgStateExec(PrgState prg) throws MyException {
    if (logFilePath == null) {
      setLogFilePath();
    }
    PrintWriter logFile;
    try {
      logFile = new PrintWriter(new BufferedWriter(new FileWriter(logFilePath, true)));
    } catch (IOException e) {
      throw new MyException("Error opening log file");
    }
    logFile.println(prg.toString());
    logFile.close();
  }

  @Override
  public List<PrgState> getPrgList() {
    return prgList;
  }

  @Override
  public void setPrgList(List<PrgState> prgList) {
    this.prgList = prgList;
  }
}