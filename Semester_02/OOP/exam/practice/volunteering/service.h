#pragma once
#include <iostream>

#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service() {}
  ~Service(){};
};
