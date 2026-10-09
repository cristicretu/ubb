#include "ui.h"

void UI::printMenu() {
  std::cout << "1. Add car\n";
  std::cout << "2. Get cars with max price\n";
  std::cout << "3. Write cars to file\n";
  std::cout << "0. Exit\n";
}

void UI::run() {
  int option;
  double maxPrice;
  std::string body_style, engine_type;
  int autonomy;
  std::string filename;
  std::vector<Car> wcars;

  while (true) {
    printMenu();
    std::cout << "Option: ";
    std::cin >> option;
    std::cin.ignore();

    if (option == 0) {
      break;
    }

    switch (option) {
      case 1:
        std::cout << "Body style: ";
        std::getline(std::cin, body_style);
        std::cout << "Engine type: ";
        std::getline(std::cin, engine_type);
        if (engine_type == "electric") {
          std::cout << "Autonomy: ";
          std::cin >> autonomy;
          service.addCar(body_style, engine_type, autonomy);
        } else {
          service.addCar(body_style, engine_type, 0);
        }
        break;
      case 2:
        std::cout << "Max price: ";
        std::cin >> maxPrice;
        wcars = service.getCarswithMaxPrice(maxPrice);
        for (auto car : wcars) {
          std::cout << car.toString() << '\n';
        }
        break;
      case 3:
        std::cout << "Filename: ";
        std::getline(std::cin, filename);
        wcars = service.getCars();
        service.writeToFile(filename, wcars);
        break;
      default:
        std::cout << "Invalid option\n";
        break;
    }
  }
}