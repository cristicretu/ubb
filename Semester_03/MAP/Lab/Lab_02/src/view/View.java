package view;

import controller.Controller;
import controller.MyException;
import model.PrgState;
import model.exp.ArithExp;
import model.exp.ConstantValue;
import model.exp.VariableExp;
import model.statement.AssignStmt;
import model.statement.CompStmt;
import model.statement.IStmt;
import model.statement.IfStmt;
import model.statement.PrintStmt;
import model.statement.VarDeclStmt;
import model.type.BoolType;
import model.type.IntType;
import model.value.BoolValue;
import model.value.IValue;
import model.value.IntValue;
import repository.IRepository;
import repository.Repository;
import utils.IDict;
import utils.IList;
import utils.IStack;
import utils.MyDict;
import utils.MyList;
import utils.MyStack;
import java.util.InputMismatchException;
import java.util.Scanner;

public class View {
  private static IStmt createExample1() {
    // int v; v=2; Print(v)
    return new CompStmt(
        new VarDeclStmt("v", new IntType()),
        new CompStmt(
            new AssignStmt("v", new ConstantValue(new IntValue(2))),
            new PrintStmt(new VariableExp("v"))));
  }

  private static IStmt createExample2() {
    // int a; int b; a=2+3*5; b=a+1; Print(b)
    return new CompStmt(
        new VarDeclStmt("a", new IntType()),
        new CompStmt(
            new VarDeclStmt("b", new IntType()),
            new CompStmt(
                new AssignStmt("a", new ArithExp('+', new ConstantValue(new IntValue(2)),
                    new ArithExp('*', new ConstantValue(new IntValue(3)), new ConstantValue(new IntValue(5))))),
                new CompStmt(
                    new AssignStmt("b", new ArithExp('+', new VariableExp("a"), new ConstantValue(new IntValue(1)))),
                    new PrintStmt(new VariableExp("b"))))));
  }

  private static IStmt createExample3() {
    // bool a; int v; a=true; (If a Then v=2 Else v=3); Print(v)
    return new CompStmt(
        new VarDeclStmt("a", new BoolType()),
        new CompStmt(
            new VarDeclStmt("v", new IntType()),
            new CompStmt(
                new AssignStmt("a", new ConstantValue(new BoolValue(true))),
                new CompStmt(
                    new IfStmt(new VariableExp("a"),
                        new AssignStmt("v", new ConstantValue(new IntValue(2))),
                        new AssignStmt("v", new ConstantValue(new IntValue(3)))),
                    new PrintStmt(new VariableExp("v"))))));
  }

  public static void main(String[] args) {
    Scanner scanner = new Scanner(System.in);
    IStmt selectedProgram = null;

    while (selectedProgram == null) {
      System.out.println("1. int v; v=2; Print(v)");
      System.out.println("2. int a; int b; a=2+3*5; b=a+1; Print(b)");
      System.out.println("3. bool a; int v; a=true; (If a Then v=2 Else v=3); Print(v)");
      System.out.println("\nSelect the program to execute: (1-3)");

      try {
        int choice = scanner.nextInt();
        switch (choice) {
          case 1:
            selectedProgram = createExample1();
            break;
          case 2:
            selectedProgram = createExample2();
            break;
          case 3:
            selectedProgram = createExample3();
            break;
          default:
            System.out.println("Invalid choice!");
        }
      } catch (InputMismatchException e) {
        System.out.println("Please enter a valid number!");
        scanner.nextLine();
      }
    }

    IStack<IStmt> stk = new MyStack<>();
    IDict<String, IValue> symTable = new MyDict<>();
    IList<IValue> output = new MyList<>();

    PrgState prg = new PrgState(stk, symTable, output, selectedProgram);
    IRepository repo = new Repository(prg);
    Controller ctrl = new Controller(repo);

    try {
      ctrl.allSteps();
    } catch (MyException e) {
      System.out.println(e.getMessage());
    }

    scanner.close();
  }
}
