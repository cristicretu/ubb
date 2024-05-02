#include "../../headers/repository/repository.h"

#include <sqlite3.h>

#include <fstream>
#include <iostream>
#include <sstream>

Repository::Repository() { this->dogs = std::vector<Dog>(); }

Repository::~Repository() {}

void Repository::addDog(const Dog &dog) { this->dogs.push_back(dog); }

void Repository::removeDog(int index) {
  this->dogs.erase(this->dogs.begin() + index);
}

void Repository::updateDog(int index, const Dog &dog) {
  this->dogs[index] = dog;
}

std::vector<Dog> Repository::getDogs() const { return this->dogs; }

int Repository::getNumberOfDogs() const { return this->dogs.size(); }

Dog Repository::getDog(int index) const { return this->dogs[index]; }

int Repository::findDog(const Dog &dog) const {
  int index =
      std::find(this->dogs.begin(), this->dogs.end(), dog) - this->dogs.begin();

  return index < this->dogs.size() ? index : -1;
}