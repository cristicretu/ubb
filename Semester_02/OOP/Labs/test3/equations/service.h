#pragma once
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service(){};
  ~Service(){};

  std::vector<Equation> getEquations() const { return repo.getEquations(); }
  void updateEquation(int index, Equation eq) {
    repo.updateEquation(index, eq);
  }
};
