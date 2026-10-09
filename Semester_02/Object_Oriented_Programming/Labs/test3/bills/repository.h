#pragma once
#include <fstream>
#include <sstream>
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Bill> bills;

 public:
  Repository() { loadData(); };
  ~Repository(){};

  void loadData() {
    std::ifstream fin("../bills.txt");
    std::string line;

    std::string name, serial, price, paid;
    while (std::getline(fin, line)) {
      std::istringstream iss(line);

      std::getline(iss, name, ';');
      std::getline(iss, serial, ';');
      std::getline(iss, price, ';');
      std::getline(iss, paid, ';');

      bills.push_back(
          Bill(name, serial, std::stod(price), paid == "false" ? 0 : 1));
    }
  }

  std::vector<Bill> getBills() const { return bills; }
};
