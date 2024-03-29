#include "domain.h"

#include <string>

Car::Car(std::string name, std::string model, std::string color, int year)
    : name(name), model(model), color(color), year(year) {}

Car::~Car() {}

std::string Car::getName() const { return this->name; }
std::string Car::getModel() const { return this->model; }
std::string Car::getColor() const { return this->color; }
int Car::getYear() const { return this->year; }

void Car::setName(std::string name) { this->name = name; }
void Car::setColor(std::string color) { this->color = color; }
void Car::setModel(std::string model) { this->model = model; }
void Car::setYear(int year) { this->year = year; }
