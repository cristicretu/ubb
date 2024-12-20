package view;

import view.command.ExitCommand;
import view.command.RunExample;
import model.PrgState;
import model.exp.ArithExp;
import model.exp.ConstantValue;
import model.exp.VariableExp;
import model.statement.AssignStmt;
import model.statement.CloseRFile;
import model.statement.CompStmt;
import model.statement.IStmt;
import model.statement.IfStmt;
import model.statement.OpenRFile;
import model.statement.PrintStmt;
import model.statement.ReadFile;
import model.statement.VarDeclStmt;
import model.type.BoolType;
import model.type.IntType;
import model.type.StringType;
import model.value.BoolValue;
import model.value.IntValue;
import model.value.StringValue;
import repository.IRepository;
import repository.Repository;
import utils.IDict;
import utils.IList;
import utils.IStack;
import utils.MyDict;
import utils.MyList;
import utils.MyStack;
import model.value.IValue;
import java.io.BufferedReader;

import controller.Controller;

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

  private static IStmt createExample4() {
    // string varf; varf = "test.in"; openRFile(varf); int varc; readFile(varf,
    // varc); Print(varc); readFile(varf, varc); Print(varc); closeRFile(varf);
    return new CompStmt(
        new VarDeclStmt("varf", new StringType()),
        new CompStmt(new AssignStmt("varf", new ConstantValue(new StringValue("test.in"))),
            new CompStmt(new OpenRFile(new VariableExp("varf")),
                new CompStmt(new VarDeclStmt("varc", new IntType()),
                    new CompStmt(new ReadFile(new VariableExp("varf"), "varc"),
                        new CompStmt(new PrintStmt(new VariableExp("varc")),
                            new CompStmt(new ReadFile(new VariableExp("varf"), "varc"),
                                new CompStmt(new PrintStmt(new VariableExp("varc")),
                                    new CloseRFile(new VariableExp("varf"))))))))));
  }

  private static PrgState createPrgState(IStmt originalProgram) {
    IStack<IStmt> exeStack = new MyStack<>();
    IDict<String, IValue> symTable = new MyDict<>();
    IList<IValue> output = new MyList<>();
    IDict<StringValue, BufferedReader> fileTable = new MyDict<>();

    return new PrgState(exeStack, symTable, output, originalProgram, fileTable);
  }

  private static Controller createController(IStmt stmt, String logFilePath) {
    PrgState prgState = createPrgState(stmt);
    IRepository repo = new Repository(prgState, logFilePath);
    return new Controller(repo);
  }

  public static void main(String[] args) {
    TextMenu menu = new TextMenu();

    menu.addCommand(new RunExample("1", createExample1(), createController(createExample1(), "log1.txt")));
    menu.addCommand(new RunExample("2", createExample2(), createController(createExample2(), "log2.txt")));
    menu.addCommand(new RunExample("3", createExample3(), createController(createExample3(), "log3.txt")));
    menu.addCommand(new RunExample("4", createExample4(), createController(createExample4(), "log4.txt")));
    menu.addCommand(new ExitCommand("0", "Exit"));

    menu.show();
  }
}
