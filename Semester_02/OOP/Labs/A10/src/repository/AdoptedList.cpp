#include "../../headers/repository/AdoptedList.h"

AdoptedList::AdoptedList() { this->dogs = std::vector<Dog>(); }

AdoptedList::~AdoptedList() {}

void AdoptedList::addDog(const Dog &dog) { this->dogs.push_back(dog); }

void AdoptedList::removeDog(int index) {
  if (index < 0 || index >= this->dogs.size()) {
    throw std::invalid_argument("Dog does not exist");
  }

  this->dogs.erase(this->dogs.begin() + index);
}

std::vector<Dog> AdoptedList::getDogs() const { return this->dogs; }

int AdoptedList::findDog(const Dog &dog) const {
  for (int i = 0; i < this->dogs.size(); i++) {
    if (this->dogs[i] == dog) {
      return i;
    }
  }

  return -1;
}
