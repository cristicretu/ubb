#pragma once
#include <iostream>

#include "service.h"

class UI {
 private:
  Service service;

 public:
  UI();
  ~UI();

  void run();
  void printMenu();
};