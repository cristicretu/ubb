#pragma once
#include <string>

class Car {
 private:
  std::string name, model, color;
  int year;

 public:
  Car(std::string name, std::string model, std::string color, int year);

  ~Car();

  std::string getName() const;
  std::string getModel() const;
  std::string getColor() const;
  int getYear() const;

  bool operator==(const Car& car) {
    return this->name == car.name && this->model == car.model &&
           this->color == car.color && this->year == car.year;
  }

  void setName(std::string name);
  void setColor(std::string color);
  void setModel(std::string model);
  void setYear(int year);
};