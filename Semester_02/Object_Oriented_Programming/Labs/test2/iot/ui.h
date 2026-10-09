#pragma once
#include "service.h"

class UI {
 private:
  Device device;

 public:
  UI(){};
  ~UI(){};

  void printMenu();
  void run();
};
