package view.command;

import controller.Controller;
import exceptions.MyException;
import model.statement.IStmt;

public class RunExample extends Command {
  private Controller controller;

  public RunExample(String key, IStmt stmt, Controller controller) {
    super(key, stmt.toString());
    this.controller = controller;
  }

  @Override
  public void execute() {
    try {
      controller.allSteps();
    } catch (MyException e) {
      System.out.println(e.getMessage());
    }
  }
}
