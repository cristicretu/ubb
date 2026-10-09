#include "ui.h"

#include <iostream>

UI::UI() { this->service = Service(); }

UI::~UI() {}

void UI::run() {
  while (true) {
    this->printMenu();
    int command;
    std::cin >> command;
    std::cin.ignore();
    if (command == 0) {
      break;
    }
    switch (command) {
      case 1:
        this->addBill();
        break;
      case 2:
        this->removeBill();
        break;
      case 3:
        this->printBills();
        break;
      case 4:
        this->printTotalUnpaid();
        break;
      case 5:
        this->sortBills();
        break;
      default:
        std::cout << "Invalid command!\n";
        break;
    }
  }
}

void UI::printMenu() {
  std::cout << "1. Add bill\n";
  std::cout << "2. Remove bill\n";
  std::cout << "3. Print bills\n";
  std::cout << "4. Print total unpaid\n";
  std::cout << "5. Sort bills\n";
  std::cout << "0. Exit\n";
}

void UI::addBill() {
  std::string serial_number;
  std::string company;
  int day;
  int month;
  int year;
  bool paid;

  std::cout << "Serial number: ";
  std::getline(std::cin, serial_number);
  std::cout << "Company: ";
  std::getline(std::cin, company);
  std::cout << "Day: ";
  std::cin >> day;
  std::cout << "Month: ";
  std::cin >> month;
  std::cout << "Year: ";
  std::cin >> year;
  std::cout << "Paid (1/0): ";
  std::cin >> paid;

  if (this->service.addBill(serial_number, company, day, month, year, paid)) {
    std::cout << "Bill added successfully!\n";
  } else {
    std::cout << "Bill could not be added!\n";
  }
}

void UI::removeBill() {
  std::string serial_number;
  std::cout << "Serial number: ";
  std::getline(std::cin, serial_number);

  if (this->service.removeBill(serial_number)) {
    std::cout << "Bill removed successfully!\n";
  } else {
    std::cout << "Bill could not be removed!\n";
  }
}

void UI::printBills() {
  std::vector<Bill> bills = this->service.getAll();
  for (int i = 0; i < bills.size(); i++) {
    std::cout << "Serial number: " << bills[i].getSerialNumber() << "\n";
    std::cout << "Company: " << bills[i].getCompany() << "\n";
    std::cout << "Date: " << bills[i].getDay() << "/" << bills[i].getMonth()
              << "/" << bills[i].getYear() << "\n";
    std::cout << "Paid: " << bills[i].getPaidStatus() << "\n";
    std::cout << "\n";
  }
}

void UI::printTotalUnpaid() {
  std::cout << "Total unpaid: " << this->service.calculateTotalUnpaid() << "\n";
}

void UI::sortBills() { this->service.sortBills(); }
