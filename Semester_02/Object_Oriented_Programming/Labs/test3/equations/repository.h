#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "domain.h"

class Repository {
 private:
  std::vector<Equation> equations;

 public:
  Repository() { loadData(); }
  ~Repository(){};

  void loadData() {
    std::ifstream fin("../eq.txt");

    std::string line;
    std::string astr, bstr, cstr;

    while (std::getline(fin, line)) {
      std::istringstream iss(line);

      std::getline(iss, astr, ',');
      std::getline(iss, bstr, ',');
      std::getline(iss, cstr, ',');

      equations.push_back(
          Equation(std::stod(astr), std::stod(bstr), std::stod(cstr)));
    }
  }

  std::vector<Equation> getEquations() const { return equations; }

  void updateEquation(int index, Equation eq) {
    if (index < 0 || index >= equations.size()) {
      return;
    }

    equations[index] = eq;
  }
};
