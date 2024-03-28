#pragma once
#include <string>
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Bill> data;

 public:
  Repository();

  ~Repository();

  bool addBill(Bill b);
  bool removeBill(Bill b);
  int getIndex(Bill b);
  std::vector<Bill> getAll() const;
  void sortBills();
};
