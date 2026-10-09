#include "repository.h"

#include "domain.h"

Repository::Repository() { this->data = std::vector<Car>(); }

Repository::~Repository() {}

void Repository::addCar(Car car) { this->data.push_back(car); }

int Repository::getIndex(Car car) {
  for (int i = 0, n = this->data.size(); i < n; ++i) {
    if (this->data[i] == car) {
      return i;
    }
  }
  return -1;
}

void Repository::removeCar(int index) {
  this->data.erase(this->data.begin() + index);
}

std::vector<Car> Repository::getAll() const { return this->data; }