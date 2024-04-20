#include "service.h"

#include <fstream>
#include <iostream>

void Service::addCar(std::string body_style, std::string engine_type,
                     int autonomy) {
  if (engine_type == "electric") {
    this->cars.push_back(Car(body_style, new ElectricEngine(autonomy)));
  } else {
    this->cars.push_back(Car(body_style, new TurboEngine()));
  }

  for (auto car : this->cars) {
    std::cout << car.toString() << '\n';
  }
}

std::vector<Car> Service::getCarswithMaxPrice(double maxPrice) {
  std::vector<Car> ans = (this->cars);

  ans.erase(
      std::remove_if(ans.begin(), ans.end(),
                     [maxPrice](Car car) { return car.getPrice() > maxPrice; }),
      ans.end());

  return ans;
}

void Service::writeToFile(std::string filename, std::vector<Car> wcars) {
  std::ofstream fout(filename);
  for (auto car : wcars) {
    fout << car.toString() << '\n';
  }
  fout.close();
}