package view;

import view.command.ExitCommand;
import view.command.RunExample;
import model.PrgState;
import model.exp.ArithExp;
import model.exp.ConstantValue;
import model.exp.RefExp;
import model.exp.RelExp;
import model.exp.VariableExp;
import model.statement.AssignStmt;
import model.statement.CloseRFile;
import model.statement.CompStmt;
import model.statement.ForkStmt;
import model.statement.IStmt;
import model.statement.IfStmt;
import model.statement.NewStmt;
import model.statement.OpenRFile;
import model.statement.PrintStmt;
import model.statement.ReadFile;
import model.statement.VarDeclStmt;
import model.statement.WhileStmt;
import model.statement.WriteHeapStmt;
import model.type.BoolType;
import model.type.IntType;
import model.type.RefType;
import model.type.StringType;
import model.value.BoolValue;
import model.value.IntValue;
import model.value.StringValue;
import repository.IRepository;
import repository.Repository;
import utils.IDict;
import utils.IHeap;
import utils.IList;
import utils.IStack;
import utils.MyDict;
import utils.MyHeap;
import utils.MyList;
import utils.MyStack;
import model.value.IValue;
import java.io.BufferedReader;

import controller.Controller;
import model.type.IType;
import exceptions.MyException;

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
                                new AssignStmt("a", new ArithExp('+',
                                        new ConstantValue(new IntValue(2)),
                                        new ArithExp('*', new ConstantValue(
                                                new IntValue(3)),
                                                new ConstantValue(
                                                        new IntValue(5))))),
                                new CompStmt(
                                        new AssignStmt("b", new ArithExp('+',
                                                new VariableExp("a"),
                                                new ConstantValue(
                                                        new IntValue(1)))),
                                        new PrintStmt(new VariableExp("b"))))));
    }

    private static IStmt createExample3() {
        // bool a; int v; a=true; (If a Then v=2 Else v=3); Print(v)
        return new CompStmt(
                new VarDeclStmt("a", new BoolType()),
                new CompStmt(
                        new VarDeclStmt("v", new IntType()),
                        new CompStmt(
                                new AssignStmt("a",
                                        new ConstantValue(new BoolValue(true))),
                                new CompStmt(
                                        new IfStmt(new VariableExp("a"),
                                                new AssignStmt("v",
                                                        new ConstantValue(
                                                                new IntValue(2))),
                                                new AssignStmt("v",
                                                        new ConstantValue(
                                                                new IntValue(3)))),
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
                                        new CompStmt(new ReadFile(
                                                new VariableExp("varf"),
                                                "varc"),
                                                new CompStmt(new PrintStmt(
                                                        new VariableExp("varc")),
                                                        new CompStmt(new ReadFile(
                                                                new VariableExp("varf"),
                                                                "varc"),
                                                                new CompStmt(new PrintStmt(
                                                                        new VariableExp("varc")),
                                                                        new CloseRFile(new VariableExp(
                                                                                "varf"))))))))));
    }

    private static IStmt createExample5() {
        // Ref int v;new(v,20);Ref Ref int a; new(a,v);print(v);print(a)
        return new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new VarDeclStmt("a",
                                        new RefType(new RefType(
                                                new IntType()))),
                                new CompStmt(
                                        new NewStmt("a", new VariableExp("v")),
                                        new CompStmt(
                                                new PrintStmt(new VariableExp(
                                                        "v")),
                                                new PrintStmt(new VariableExp(
                                                        "a")))))));
    }

    private static IStmt createExample6() {
        // Ref int v;new(v,20);Ref Ref int a; new(a,v);print(rH(v));print(rH(rH(a))+5)
        return new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new VarDeclStmt("a",
                                        new RefType(new RefType(
                                                new IntType()))),
                                new CompStmt(
                                        new NewStmt("a", new VariableExp("v")),
                                        new CompStmt(
                                                new PrintStmt(new RefExp(
                                                        new VariableExp("v"))),
                                                new PrintStmt(new ArithExp(
                                                        '+',
                                                        new RefExp(new RefExp(
                                                                new VariableExp("a"))),
                                                        new ConstantValue(
                                                                new IntValue(5)))))))));
    }

    private static IStmt createExample7() {
        // Ref int v;new(v,20);print(rH(v)); wH(v,30);print(rH(v)+5);
        return new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new PrintStmt(new RefExp(new VariableExp("v"))),
                                new CompStmt(
                                        new WriteHeapStmt("v",
                                                new ConstantValue(
                                                        new IntValue(30))),
                                        new PrintStmt(new ArithExp('+',
                                                new RefExp(new VariableExp(
                                                        "v")),
                                                new ConstantValue(
                                                        new IntValue(5))))))));
    }

    private static IStmt createExample8() {
        // Ref int v;new(v,20);Ref Ref int a; new(a,v); new(v,30);print(rH(rH(a)))
        return new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new VarDeclStmt("a",
                                        new RefType(new RefType(
                                                new IntType()))),
                                new CompStmt(
                                        new NewStmt("a", new VariableExp("v")),
                                        new CompStmt(
                                                new NewStmt("v", new ConstantValue(
                                                        new IntValue(30))),
                                                new PrintStmt(new RefExp(
                                                        new RefExp(new VariableExp(
                                                                "a")))))))));
    }

    private static IStmt createExample9() {
        // int v; v=4; (while (v>0) print(v);v=v-1);print(v)
        return new CompStmt(
                new VarDeclStmt("v", new IntType()),
                new CompStmt(
                        new AssignStmt("v", new ConstantValue(new IntValue(4))),
                        new CompStmt(
                                new WhileStmt(
                                        new RelExp(
                                                new VariableExp("v"),
                                                new ConstantValue(
                                                        new IntValue(0)),
                                                ">"),
                                        new CompStmt(
                                                new PrintStmt(new VariableExp(
                                                        "v")),
                                                new AssignStmt("v",
                                                        new ArithExp('-',
                                                                new VariableExp("v"),
                                                                new ConstantValue(
                                                                        new IntValue(1)))))),
                                new PrintStmt(new VariableExp("v")))));
    }

    private static IStmt createExample10() {
        // int v; Ref int a; v=10; new(a,22);
        // fork(wH(a,30);v=32;print(v);print(rH(a)));
        // print(v);print(rH(a))
        return new CompStmt(
                new VarDeclStmt("v", new IntType()),
                new CompStmt(
                        new VarDeclStmt("a", new RefType(new IntType())),
                        new CompStmt(
                                new AssignStmt("v",
                                        new ConstantValue(new IntValue(10))),
                                new CompStmt(
                                        new NewStmt("a", new ConstantValue(
                                                new IntValue(22))),
                                        new CompStmt(
                                                new ForkStmt(
                                                        new CompStmt(
                                                                new WriteHeapStmt(
                                                                        "a",
                                                                        new ConstantValue(
                                                                                new IntValue(30))),
                                                                new CompStmt(
                                                                        new AssignStmt("v",
                                                                                new ConstantValue(
                                                                                        new IntValue(32))),
                                                                        new CompStmt(
                                                                                new PrintStmt(new VariableExp(
                                                                                        "v")),
                                                                                new PrintStmt(new RefExp(
                                                                                        new VariableExp("a"))))))),
                                                new CompStmt(
                                                        new PrintStmt(new VariableExp(
                                                                "v")),
                                                        new PrintStmt(new RefExp(
                                                                new VariableExp("a")))))))));
    }

    private static IStmt createExampleWithTypeError() {
        // bool a; int b; b = a + 5;
        return new CompStmt(
                new VarDeclStmt("a", new BoolType()),
                new CompStmt(
                        new VarDeclStmt("b", new IntType()),
                        new AssignStmt("b",
                                new ArithExp('+',
                                        new VariableExp("a"),
                                        new ConstantValue(new IntValue(5))))));
    }

    private static PrgState createPrgState(IStmt originalProgram) throws MyException {
        IDict<String, IType> typeEnv = new MyDict<>();

        originalProgram.typecheck(typeEnv);

        IStack<IStmt> exeStack = new MyStack<>();
        IDict<String, IValue> symTable = new MyDict<>();
        IList<IValue> output = new MyList<>();
        IDict<StringValue, BufferedReader> fileTable = new MyDict<>();
        IHeap<Integer, IValue> heap = new MyHeap<>();

        return new PrgState(exeStack, symTable, output, originalProgram, fileTable, heap);
    }

    private static Controller createController(IStmt stmt, String logFilePath) throws MyException {
        PrgState prgState = createPrgState(stmt);
        IRepository repo = new Repository(prgState, logFilePath);
        return new Controller(repo);
    }

    public static void main(String[] args) {
        TextMenu menu = new TextMenu();

        try {
            menu.addCommand(new RunExample("1", createExample1(),
                    createController(createExample1(), "log1.txt")));
            menu.addCommand(new RunExample("2", createExample2(),
                    createController(createExample2(), "log2.txt")));
            menu.addCommand(new RunExample("3", createExample3(),
                    createController(createExample3(), "log3.txt")));
            menu.addCommand(new RunExample("4", createExample4(),
                    createController(createExample4(), "log4.txt")));
            menu.addCommand(new RunExample("5", createExample5(),
                    createController(createExample5(), "log5.txt")));
            menu.addCommand(new RunExample("6", createExample6(),
                    createController(createExample6(), "log6.txt")));
            menu.addCommand(new RunExample("7", createExample7(),
                    createController(createExample7(), "log7.txt")));
            menu.addCommand(new RunExample("8", createExample8(),
                    createController(createExample8(), "log8.txt")));
            menu.addCommand(new RunExample("9", createExample9(),
                    createController(createExample9(), "log9.txt")));
            menu.addCommand(new RunExample("10", createExample10(),
                    createController(createExample10(), "log10.txt")));
        } catch (MyException e) {
            System.out.println("Error during program initialization: " + e.getMessage());
            System.exit(1);
        }

        menu.addCommand(new ExitCommand("0", "Exit"));
        menu.show();
    }
}
