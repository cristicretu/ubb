package view.gui;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import model.statement.IStmt;
import model.statement.CompStmt;
import model.statement.AssignStmt;
import model.statement.PrintStmt;
import model.statement.IfStmt;
import model.exp.VariableExp;
import java.util.ArrayList;
import java.util.List;

public class ProgramListController {
  @FXML
  private ListView<String> programListView;
  private List<IStmt> programs;

  @FXML
  public void initialize() {
    programs = new ArrayList<>();
    // Add your example programs here
    // Example program 1: v=2;Print(v)
    IStmt prog1 = new CompStmt(
        new AssignStmt("v", new VariableExp("2")),
        new PrintStmt(new VariableExp("v")));
    programs.add(prog1);

    // Convert programs to their string representations
    ObservableList<String> programStrings = FXCollections.observableArrayList();
    for (IStmt stmt : programs) {
      programStrings.add(stmt.toString());
    }

    programListView.setItems(programStrings);
  }

  @FXML
  private void executeProgram() {
    int selectedIndex = programListView.getSelectionModel().getSelectedIndex();
    if (selectedIndex >= 0) {
      IStmt selectedProgram = programs.get(selectedIndex);
      // TODO: Execute the selected program using the interpreter
    } else {
      Alert alert = new Alert(AlertType.WARNING);
      alert.setTitle("Warning");
      alert.setHeaderText(null);
      alert.setContentText("Please select a program to execute!");
      alert.showAndWait();
    }
  }
}