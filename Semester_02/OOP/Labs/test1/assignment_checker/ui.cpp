#include "ui.h"

#include <iostream>

UI::UI() { this->service = Service(); }

UI::~UI() {}

void UI::run() {
  while (true) {
    printMenu();
    int command;
    std::cin >> command;
    std::cin.ignore();
    if (command == 0) {
      break;
    }
    if (command == 1) {
      std::string name, solution;
      int id;
      std::cout << "Name: ";
      std::getline(std::cin, name);
      std::cout << "Solution: ";
      std::getline(std::cin, solution);
      std::cout << "ID: ";
      std::cin >> id;
      std::cin.ignore();
      service.addAssignment(name, solution, id);
    }
    if (command == 2) {
      std::vector<Assignment> assignments = service.getAssignments();
      for (const auto& assignment : assignments) {
        std::cout << assignment.getName() << " " << assignment.getSolution()
                  << " " << assignment.getId() << "\n";
      }
    }
    if (command == 3) {
      std::vector<std::vector<Assignment>> result = service.dishnestyCheck();
      for (const auto& group : result) {
        for (const auto& assignment : group) {
          std::cout << assignment.getName() << " " << assignment.getSolution()
                    << " " << assignment.getId() << "\n";
        }
        std::cout << "\n";
      }
    }
  }
}

void UI::printMenu() {
  std::cout << "1. Add assignment\n";
  std::cout << "2. Get assignments\n";
  std::cout << "3. Dishonesty check\n";
  std::cout << "0. Exit\n";
}