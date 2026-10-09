#pragma once
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Task> tasks;

 public:
  Repository() { loadData(); }
  ~Repository(){};

  void loadData();
  std::vector<Task> getTasks() const;
};
