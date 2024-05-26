#pragma once
#include <algorithm>

#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service(){};
  ~Service(){};

  std::vector<Bill> getBills() const {
    auto bills = repo.getBills();
    std::sort(bills.begin(), bills.end(), [](const Bill &a, const Bill &b) {
      return a.getName() < b.getName();
    });
    return bills;
  }

  std::vector<Bill> filterBills(bool paid = 1) const {
    auto bills = repo.getBills();

    bills.erase(std::remove_if(bills.begin(), bills.end(),
                               [paid](const Bill &bill) {
                                 return bill.getIsPaid() != paid;
                               }),
                bills.end());

    return bills;
  }

  int getTotal(std::string company) {
    auto bills = repo.getBills();
    int total = 0;
    bool found = false;

    for (const auto &bill : bills) {
      if (bill.getName() == company) {
        total += bill.getSum();
        found = true;
      }
    }

    if (!found) return -1;
    return total;
  }
};
