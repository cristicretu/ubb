package view;

import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

import view.command.Command;
import view.command.RunExample;

public class TextMenu {
  private Map<String, Command> commands;

  public TextMenu() {
    commands = new HashMap<>();
  }

  public void addCommand(Command c) {
    commands.put(c.getKey(), c);
  }

  public void printMenu() {
    for (Command c : commands.values()) {
      String status = "";
      if (c instanceof RunExample) {
        status = ((RunExample) c).hasBeenExecuted() ? " (executed)" : " (not executed)";
      }
      String line = String.format("%4s : %s%s", c.getKey(), c.getDescription(), status);
      System.out.println(line);
    }
  }

  public void show() {
    try (Scanner scanner = new Scanner(System.in)) {
      while (true) {
        printMenu();
        System.out.println("Input the option: ");
        String key = scanner.nextLine();
        Command c = commands.get(key);
        if (c == null) {
          System.out.println("Invalid option");
        } else {
          c.execute();
        }
      }
    }
  }
}
