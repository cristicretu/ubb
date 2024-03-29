#include "service.h"

#include "repository.h"

Service::Service() { this->repo = Repository(); }

Service::~Service() {}

void Service::addCar(std::string name, std::string model, std::string color,
                     int year) {
  Car car = Car(name, model, color, year);
  int index = this->repo.getIndex(car);

  if (index == -1) {
    this->repo.addCar(car);
  } else {
    throw std::runtime_error("Car already exists!");
  }
}

void Service::removeCar(std::string name, std::string model, std::string color,
                        int year) {
  Car car = Car(name, model, color, year);
  int index = this->repo.getIndex(car);

  if (index == -1) {
    throw std::runtime_error("Car does not exist!");
  } else {
    this->repo.removeCar(index);
  }
}

std::vector<Car> Service::getAllCars() const {
  std::vector<Car> sorted = this->repo.getAll();

  std::sort(sorted.begin(), sorted.end(), [](Car a, Car b) {
    return a.getModel() < b.getModel() ||
           (a.getModel() == b.getModel() && a.getName() < b.getName());
  });

  return sorted;
}

std::vector<Car> Service::getVintage() const {
  std::vector<Car> cars = this->repo.getAll();
  std::vector<Car> vintage;

  for (Car car : cars) {
    if (car.getYear() < 1980) {
      vintage.push_back(car);
    }
  }

  return vintage;
}