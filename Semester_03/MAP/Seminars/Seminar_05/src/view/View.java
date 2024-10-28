package view;

import controller.Controller;
import controller.MyException;
import model.PrgState;
import model.exp.ValueExp;
import model.statement.CompStmt;
import model.statement.IStmt;
import model.statement.PrintStmt;
import model.statement.VarDeclStmt;
import model.type.IntType;
import model.value.Value;
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
        new PrintStmt(new ValueExp("v")));

    IStack<IStmt> stk = new MyStack<>();
    IDict<String, Value> symTable = new MyDict<>();
    IList<Value> output = new MyList<>();

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
