#include "ui.h"

#include <iostream>

void UI::printMenu() {
  std::cout << "1. Add sensor\n";
  std::cout << "2. Get all sensors\n";
  std::cout << "3. Get alerts\n";
  std::cout << "4. Write to file\n";
  std::cout << "0. Exit\n";
}

void UI::run() {
  int cmd = -1;
  while (cmd != 0) {
    printMenu();
    std::cout << "Enter command: ";
    std::cin >> cmd;
    if (cmd == 1) {
      std::string type, producer;
      std::vector<double> recordings;
      double diameter, length;
      std::cout << "Enter type: ";
      std::cin >> type;
      std::cout << "Enter producer: ";
      std::cin >> producer;
      std::cout << "Enter recordings: ";
      double rec;
      while (std::cin >> rec) {
        recordings.push_back(rec);
      }
      std::cout << "Enter diameter: ";
      std::cin >> diameter;
      std::cout << "Enter length: ";
      std::cin >> length;
      device.addSensor(type, producer, recordings, diameter, length);
    } else if (cmd == 2) {
      std::vector<Sensor*> sensors = device.getSensors();
      for (auto s : sensors) {
        std::cout << s->toString();
      }
    } else if (cmd == 3) {
      std::vector<Sensor*> alerts = device.getAlerts();
      for (auto a : alerts) {
        std::cout << a->toString();
      }
    } else if (cmd == 4) {
      std::string filename;
      std::cout << "Enter filename: ";
      std::cin >> filename;
      device.writeToFile(filename);
    }
  }
}