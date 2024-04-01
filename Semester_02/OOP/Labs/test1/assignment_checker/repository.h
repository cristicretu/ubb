#pragma once
#include <domain.h>

#include <vector>

class Repository {
 private:
  std::vector<Assignment> data;

 public:
  Repository();
  ~Repository();
  void addAssignment(Assignment a);
  std::vector<Assignment> getAssignments() const;
  void readFromFile(std::string filename);
};