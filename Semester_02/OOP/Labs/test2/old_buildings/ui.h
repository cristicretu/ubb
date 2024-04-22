#pragma once
#include "service.h"

class UI {
 private:
  Service service;

 public:
  UI(){};
  ~UI(){};

  void printMenu();
  void run();
};
