#pragma once
#include "domain.h"
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service();

  ~Service();

  void addCar(std::string name, std::string model, std::string color, int year);
  void removeCar(std::string name, std::string model, std::string color,
                 int year);

  std::vector<Car> getAllCars() const;
  std::vector<Car> getVintage() const;
};