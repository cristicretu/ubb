#pragma once
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Car> data;

 public:
  Repository();
  ~Repository();

  void addCar(Car car);
  void removeCar(int index);
  int getIndex(Car car);
  std::vector<Car> getAll() const;
};