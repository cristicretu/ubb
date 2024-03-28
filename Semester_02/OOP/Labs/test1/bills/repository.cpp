#include "repository.h"

Repository::Repository() { this->data = std::vector<Bill>(); }

Repository::~Repository() {}

bool Repository::addBill(Bill b) {
  this->data.push_back(b);
  return true;
}

bool Repository::removeBill(Bill b) {
  for (int i = 0; i < this->data.size(); i++) {
    if (this->data[i].getSerialNumber() == b.getSerialNumber()) {
      this->data.erase(this->data.begin() + i);
      return true;
    }
  }

  return false;
}

int Repository::getIndex(Bill b) {
  for (int i = 0; i < this->data.size(); ++i) {
    if (this->data[i].getSerialNumber() == b.getSerialNumber()) {
      return i;
    }
  }

  return -1;
}

std::vector<Bill> Repository::getAll() const { return this->data; }

void Repository::sortBills() {
  for (int i = 0; i < this->data.size() - 1; i++) {
    for (int j = i + 1; j < this->data.size(); j++) {
      if (this->data[i].getYear() > this->data[j].getYear()) {
        std::swap(this->data[i], this->data[j]);
      } else if (this->data[i].getYear() == this->data[j].getYear()) {
        if (this->data[i].getMonth() > this->data[j].getMonth()) {
          std::swap(this->data[i], this->data[j]);
        } else if (this->data[i].getMonth() == this->data[j].getMonth()) {
          if (this->data[i].getDay() > this->data[j].getDay()) {
            std::swap(this->data[i], this->data[j]);
          }
        }
      }
    }
  }
}
