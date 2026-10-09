#include "ui.h"

UI::UI() { this->service = Service(); }

UI::~UI() {}

void UI::printMenu() {
  std::cout << "1. Add car\n";
  std::cout << "2. Remove car\n";
  std::cout << "3. Show all cars\n";
  std::cout << "4. Show vintage cars\n";
  std::cout << "0. Exit\n";
}

void UI::run() {
  while (true) {
    this->printMenu();

    int option;
    std::cout << "Option: ";
    std::cin >> option;

    if (option == 0) {
      break;
    }

    if (option == 1) {
      std::string name, model, color;
      int year;

      std::cout << "Name: ";
      std::cin >> name;

      std::cout << "Model: ";
      std::cin >> model;

      std::cout << "Color: ";
      std::cin >> color;

      std::cout << "Year: ";
      std::cin >> year;

      try {
        this->service.addCar(name, model, color, year);
      } catch (std::runtime_error &e) {
        std::cout << e.what() << '\n';
      }
    }

    if (option == 2) {
      std::string name, model, color;
      int year;

      std::cout << "Name: ";
      std::cin >> name;

      std::cout << "Model: ";
      std::cin >> model;

      std::cout << "Color: ";
      std::cin >> color;

      std::cout << "Year: ";
      std::cin >> year;

      try {
        this->service.removeCar(name, model, color, year);
      } catch (std::runtime_error &e) {
        std::cout << e.what() << '\n';
      }
    }

    if (option == 3) {
      std::vector<Car> cars = this->service.getAllCars();

      for (Car car : cars) {
        std::cout << car.getName() << ' ' << car.getModel() << ' '
                  << car.getColor() << ' ' << car.getYear() << '\n';
      }
    }

    if (option == 4) {
      std::vector<Car> cars = this->service.getVintage();

      for (Car car : cars) {
        std::cout << car.getName() << ' ' << car.getModel() << ' '
                  << car.getColor() << ' ' << car.getYear() << '\n';
      }
    }
  }
}