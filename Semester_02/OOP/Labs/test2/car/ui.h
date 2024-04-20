#pragma once
#include <iostream>

#include "service.h"

class UI {
 private:
  Service service;

 public:
  UI() { this->service = Service(); };
  ~UI(){};
  void printMenu();
  void run();
};