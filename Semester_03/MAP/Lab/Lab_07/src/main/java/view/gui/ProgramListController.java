package view.gui;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.scene.Parent;
import javafx.scene.control.ListView;
import javafx.scene.control.Alert;
import model.statement.IStmt;
import model.statement.CompStmt;
import model.statement.AssignStmt;
import model.statement.PrintStmt;
import model.statement.IfStmt;
import model.statement.VarDeclStmt;
import model.statement.WhileStmt;
import model.statement.NewStmt;
import model.statement.WriteHeapStmt;
import model.statement.ForkStmt;
import model.statement.OpenRFile;
import model.statement.ReadFile;
import model.statement.CloseRFile;
import model.exp.VariableExp;
import model.exp.ArithExp;
import model.exp.RelExp;
import model.exp.ConstantValue;
import model.exp.RefExp;
import model.type.IntType;
import model.type.BoolType;
import model.type.RefType;
import model.type.StringType;
import model.value.IntValue;
import model.value.BoolValue;
import model.value.StringValue;
import model.PrgState;
import repository.IRepository;
import repository.Repository;
import controller.Controller;
import utils.MyStack;
import utils.MyDict;
import utils.MyList;
import utils.MyHeap;
import java.util.ArrayList;
import java.util.List;

public class ProgramListController {
    @FXML
    private ListView<String> programListView;
    private List<IStmt> programs;

    @FXML
    public void initialize() {
        programs = new ArrayList<>();

        // Example 1: int v; v=2; Print(v)
        IStmt prog1 = new CompStmt(
                new VarDeclStmt("v", new IntType()),
                new CompStmt(
                        new AssignStmt("v", new ConstantValue(new IntValue(2))),
                        new PrintStmt(new VariableExp("v"))));
        programs.add(prog1);

        // Example 2: int a; int b; a=2+3*5; b=a+1; Print(b)
        IStmt prog2 = new CompStmt(
                new VarDeclStmt("a", new IntType()),
                new CompStmt(
                        new VarDeclStmt("b", new IntType()),
                        new CompStmt(
                                new AssignStmt("a",
                                        new ArithExp('+', new ConstantValue(new IntValue(2)),
                                                new ArithExp('*', new ConstantValue(new IntValue(3)),
                                                        new ConstantValue(new IntValue(5))))),
                                new CompStmt(
                                        new AssignStmt("b",
                                                new ArithExp('+', new VariableExp("a"),
                                                        new ConstantValue(new IntValue(1)))),
                                        new PrintStmt(new VariableExp("b"))))));
        programs.add(prog2);

        // Example 3: bool a; int v; a=true; If a Then v=2 Else v=3; Print(v)
        IStmt prog3 = new CompStmt(
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
        programs.add(prog3);

        // Example 4: string varf; varf = "test.in"; openRFile(varf); int varc;
        // readFile(varf, varc); Print(varc); readFile(varf, varc); Print(varc);
        // closeRFile(varf);
        IStmt prog4 = new CompStmt(
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
        programs.add(prog4);

        // Example 5: Ref int v;new(v,20);Ref Ref int a; new(a,v);print(v);print(a)
        IStmt prog5 = new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new VarDeclStmt("a", new RefType(new RefType(new IntType()))),
                                new CompStmt(
                                        new NewStmt("a", new VariableExp("v")),
                                        new CompStmt(
                                                new PrintStmt(new VariableExp("v")),
                                                new PrintStmt(new VariableExp("a")))))));
        programs.add(prog5);

        // Example 6: Ref int v;new(v,20);print(rH(v)); wH(v,30);print(rH(v)+5);
        IStmt prog6 = new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new PrintStmt(new RefExp(new VariableExp("v"))),
                                new CompStmt(
                                        new WriteHeapStmt("v", new ConstantValue(new IntValue(30))),
                                        new PrintStmt(
                                                new ArithExp('+', new RefExp(new VariableExp("v")),
                                                        new ConstantValue(new IntValue(5))))))));
        programs.add(prog6);

        // Example 7: Ref int v;new(v,20);Ref Ref int a; new(a,v);
        // new(v,30);print(rH(rH(a)))
        IStmt prog7 = new CompStmt(
                new VarDeclStmt("v", new RefType(new IntType())),
                new CompStmt(
                        new NewStmt("v", new ConstantValue(new IntValue(20))),
                        new CompStmt(
                                new VarDeclStmt("a", new RefType(new RefType(new IntType()))),
                                new CompStmt(
                                        new NewStmt("a", new VariableExp("v")),
                                        new CompStmt(
                                                new NewStmt("v", new ConstantValue(new IntValue(30))),
                                                new PrintStmt(new RefExp(new RefExp(new VariableExp("a")))))))));
        programs.add(prog7);

        // Example 9: int v; v=4; (while (v>0) print(v);v=v-1);print(v)
        IStmt prog9 = new CompStmt(
                new VarDeclStmt("v", new IntType()),
                new CompStmt(
                        new AssignStmt("v", new ConstantValue(new IntValue(4))),
                        new CompStmt(
                                new WhileStmt(
                                        new RelExp(new VariableExp("v"), new ConstantValue(new IntValue(0)), ">"),
                                        new CompStmt(
                                                new PrintStmt(new VariableExp("v")),
                                                new AssignStmt("v",
                                                        new ArithExp('-', new VariableExp("v"),
                                                                new ConstantValue(new IntValue(1)))))),
                                new PrintStmt(new VariableExp("v")))));
        programs.add(prog9);

        // Example 10: int v; Ref int a; v=10; new(a,22);
        // fork(wH(a,30);v=32;print(v);print(rH(a)));
        // print(v);print(rH(a))
        IStmt prog10 = new CompStmt(
                new VarDeclStmt("v", new IntType()),
                new CompStmt(
                        new VarDeclStmt("a", new RefType(new IntType())),
                        new CompStmt(
                                new AssignStmt("v", new ConstantValue(new IntValue(10))),
                                new CompStmt(
                                        new NewStmt("a", new ConstantValue(new IntValue(22))),
                                        new CompStmt(
                                                new ForkStmt(
                                                        new CompStmt(
                                                                new WriteHeapStmt("a",
                                                                        new ConstantValue(new IntValue(30))),
                                                                new CompStmt(
                                                                        new AssignStmt("v",
                                                                                new ConstantValue(new IntValue(32))),
                                                                        new CompStmt(
                                                                                new PrintStmt(new VariableExp("v")),
                                                                                new PrintStmt(new RefExp(
                                                                                        new VariableExp("a"))))))),
                                                new CompStmt(
                                                        new PrintStmt(new VariableExp("v")),
                                                        new PrintStmt(new RefExp(new VariableExp("a")))))))));
        programs.add(prog10);

        ObservableList<String> programStrings = FXCollections.observableArrayList();
        for (IStmt stmt : programs) {
            programStrings.add(stmt.toString());
        }

        programListView.setItems(programStrings);
    }

    @FXML
    private void executeProgram() {
        String selectedProgram = programListView.getSelectionModel().getSelectedItem();
        if (selectedProgram == null) {
            Alert alert = new Alert(Alert.AlertType.ERROR);
            alert.setTitle("Error");
            alert.setHeaderText(null);
            alert.setContentText("No program selected!");
            alert.showAndWait();
            return;
        }

        int index = programListView.getSelectionModel().getSelectedIndex();
        IStmt program = programs.get(index);

        try {
            PrgState prgState = new PrgState(
                    new MyStack<>(),
                    new MyDict<>(),
                    new MyList<>(),
                    program,
                    new MyDict<>(),
                    new MyHeap<>());

            IRepository repo = new Repository(prgState, "log" + (index + 1) + ".txt");
            Controller controller = new Controller(repo);

            // Create and show the main window
            FXMLLoader loader = new FXMLLoader(getClass().getResource("/fxml/MainWindow.fxml"));
            Parent root = loader.load();
            MainWindowController mainWindowController = loader.getController();
            mainWindowController.setController(controller);

            Stage stage = new Stage();
            stage.setTitle("Program Execution");
            stage.setScene(new Scene(root, 800, 600));
            stage.show();

            ((Stage) programListView.getScene().getWindow()).close();

        } catch (Exception e) {
            Alert alert = new Alert(Alert.AlertType.ERROR);
            alert.setTitle("Error");
            alert.setHeaderText(null);
            alert.setContentText("Error creating program state: " + e.getMessage());
            alert.showAndWait();
        }
    }
}