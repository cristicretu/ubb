package view;

import controller.Controller;
import controller.MyException;
import model.PrgState;
import model.exp.VariableExp;
import model.statement.CompStmt;
import model.statement.IStmt;
import model.statement.PrintStmt;
import model.statement.VarDeclStmt;
import model.type.IntType;
import model.value.IValue;
import repository.IRepository;
import repository.Repository;
import utils.IDict;
import utils.IList;
import utils.IStack;
import utils.MyDict;
import utils.MyList;
import utils.MyStack;

public class View {
  public static void main(String[] args) {
    IStmt ex1 = new CompStmt(
        new VarDeclStmt("v", new IntType()),
        new PrintStmt(new VariableExp("v")));

    IStack<IStmt> stk = new MyStack<>();
    IDict<String, IValue> symTable = new MyDict<>();
    IList<IValue> output = new MyList<>();

    PrgState prg = new PrgState(stk, symTable, output, ex1);

    IRepository repo = new Repository(prg);
    Controller ctrl = new Controller(repo);

    try {
      ctrl.allSteps();
    } catch (MyException e) {
      System.out.println(e.getMessage());
    }
  }
}
