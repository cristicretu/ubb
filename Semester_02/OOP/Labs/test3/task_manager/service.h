#pragma once
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service(){};
  ~Service(){};

  std::vector<Task> getAllTasks() const;
  std::pair<int, std::vector<Task>> getDurationAndTaskByPriority(
      int priority) const;
};
