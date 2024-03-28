#pragma once

#include <string>

#include "service.h"

class UI {
 private:
  Service service;

 public:
  UI();

  ~UI();

  void run();
  void printMenu();
  void addBill();
  void removeBill();
  void printBills();
  void printTotalUnpaid();
  void sortBills();
};