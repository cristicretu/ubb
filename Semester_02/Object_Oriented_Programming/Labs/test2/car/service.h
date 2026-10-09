#pragma once
#include <vector>

#include "domain.h"

class Service {
 private:
  std::vector<Car> cars;

 public:
  Service() {
    // this->cars.push_back(Car("sedan", new ElectricEngine(100)));
    // this->cars.push_back(Car("convertible", new TurboEngine()));
    // this->cars.push_back(Car("sedan", new ElectricEngine(800)));
  }
  ~Service() {}

  void addCar(std::string body_style, std::string engine_type, int autonomy);
  std::vector<Car> getCarswithMaxPrice(double maxPrice);
  void writeToFile(std::string filename, std::vector<Car> wcars);
  std::vector<Car> getCars() { return this->cars; }
};