#pragma once
#include "domain.h"
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service();
  ~Service();
  void addAssignment(std::string name, std::string solution, int id);
  std::vector<Assignment> getAssignments() const;
  std::vector<std::vector<Assignment>> dishnestyCheck() const;
};