#include "ui.h"

#include <iostream>

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
      std::string type, hospitalName;
      int numberOfDoctors, numberOfPatients, numberOfMothers, numberOfNewborns;
      double averageGrade;
      std::cout << "Type: ";
      std::getline(std::cin, type);
      std::cout << "Hospital name: ";
      std::getline(std::cin, hospitalName);
      std::cout << "Number of doctors: ";
      std::cin >> numberOfDoctors;
      std::cout << "Number of patients: ";
      std::cin >> numberOfPatients;
      if (type == "Neonatal Unit") {
        std::cout << "Number of mothers: ";
        std::cin >> numberOfMothers;
        std::cout << "Number of newborns: ";
        std::cin >> numberOfNewborns;
        std::cout << "Average grade: ";
        std::cin >> averageGrade;
      }
      service.addDepartment(type, hospitalName, numberOfDoctors,
                            numberOfPatients, numberOfMothers, numberOfNewborns,
                            averageGrade);
    }
    if (command == 2) {
      std::vector<HospitalDepartment*> efficientDepartments =
          service.getEfficientDepartments();
      for (auto department : efficientDepartments) {
        std::cout << department->toString() << "\n";
      }
    }
    if (command == 3) {
      std::string filename;
      std::cout << "Filename: ";
      std::getline(std::cin, filename);
      service.writeToFile(filename);
    }
  }
}

void UI::printMenu() {
  std::cout << "1. Add department\n";
  std::cout << "2. Show efficient departments\n";
  std::cout << "3. Write to file\n";
  std::cout << "0. Exit\n";
}