package view;

import controller.Controller;
import model.statement.IStmt;
import model.statement.CompStmt;

public class View {
  public static void main(String[] args) {
    IStmt ex1 = new CompStmt(new VarDeclStmt("v", new ValueExp(new IntValue(2))));
  }
}
