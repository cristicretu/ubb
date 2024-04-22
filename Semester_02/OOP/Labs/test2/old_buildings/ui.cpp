#include "ui.h"

#include <iostream>

void UI::printMenu() {
  std::cout << "1. Print all buildings\n";
  std::cout << "2. Print buildings to restore\n";
  std::cout << "3. Write buildings to file\n";
  std::cout << "0. Exit\n";
}

void UI::run() {
  int option;
  do {
    printMenu();
    std::cout << "Option: ";
    std::cin >> option;
    switch (option) {
      case 1: {
        for (auto b : service.getBuildings()) {
          std::cout << b->toString() << std::endl;
        }
        break;
      }
      case 2: {
        int year;
        std::cout << "Year: ";
        std::cin >> year;
        for (auto b : service.getBuildingsToRestore(year)) {
          std::cout << b->toString() << std::endl;
        }
        break;
      }
      case 3: {
        std::string filename;
        std::cout << "Filename: ";
        std::cin >> filename;
        service.writeToFile(filename);
        break;
      }
      case 0: {
        std::cout << "Exiting...\n";
        break;
      }
      default: {
        std::cout << "Invalid option\n";
        break;
      }
    }
  } while (option != 0);
}