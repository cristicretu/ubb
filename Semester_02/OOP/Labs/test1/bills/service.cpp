#include "service.h"

#include "repository.h"

Service::Service() { this->repo = Repository(); }

Service::~Service() {}

bool Service::addBill(std::string serial_number, std::string company, int day,
                      int month, int year, bool paid) {
  Bill b = Bill(serial_number, company, day, month, year, paid);
  return this->repo.addBill(b);
}

bool Service::removeBill(std::string serial_number) {
  Bill b = Bill(serial_number, "", 0, 0, 0, false);
  return this->repo.removeBill(b);
}

std::vector<Bill> Service::getAll() const { return this->repo.getAll(); }

void Service::sortBills() { this->repo.sortBills(); }

int Service::calculateTotalUnpaid() {
  int total = 0;
  std::vector<Bill> bills = this->repo.getAll();
  for (int i = 0; i < bills.size(); i++) {
    if (!bills[i].getPaidStatus()) {
      total++;
    }
  }

  return total;
}