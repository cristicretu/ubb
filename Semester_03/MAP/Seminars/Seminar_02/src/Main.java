// 1. Intr-o parcare exista masini, motociclete 
// si biciclete. Sa se afiseze toate vehiculele 
// de culoare rosie.

import repository.inMemoryRepo;
import service.service;
import ui.ui;

public class Main {

  public static void main(String[] args) {
    inMemoryRepo repo = new inMemoryRepo();
    service service = new service(repo);
    ui ui = new ui(service);
    ui.run();
  }
}
