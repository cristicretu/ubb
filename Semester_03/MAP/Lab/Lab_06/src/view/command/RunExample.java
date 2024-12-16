package view.command;

import controller.Controller;
import exceptions.MyException;
import model.statement.IStmt;

public class RunExample extends Command {
  private Controller controller;
  private boolean hasBeenExecuted;

  public RunExample(String key, IStmt stmt, Controller controller) {
    super(key, stmt.toString());
    this.controller = controller;
    this.hasBeenExecuted = false;
  }

  @Override
  public void execute() {
    if (hasBeenExecuted) {
      System.out.println("Program has already been executed!");
      return;
    }
    try {
      controller.allSteps();
      hasBeenExecuted = true;
    } catch (MyException e) {
      System.out.println(e.getMessage());
    }
  }

  public boolean hasBeenExecuted() {
    return hasBeenExecuted;
  }
}
