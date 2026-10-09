#pragma once
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service();

  ~Service();

  bool addBill(std::string serial_number, std::string company, int day,
               int month, int year, bool paid);
  bool removeBill(std::string serial_number);

  std::vector<Bill> getAll() const;

  void sortBills();

  int calculateTotalUnpaid();
};
