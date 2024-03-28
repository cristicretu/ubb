#include "../../headers/repository/repository.h"

Repository::Repository() { this->dogs = DynArray<Dog>(); }

Repository::~Repository() {}

void Repository::addDog(const Dog &dog) { this->dogs.add(dog); }

void Repository::removeDog(int index) { this->dogs.remove(index); }

void Repository::updateDog(int index, const Dog &dog) {
  this->dogs.set(index, dog);
}

DynArray<Dog> Repository::getDogs() const { return this->dogs; }

int Repository::getNumberOfDogs() const { return this->dogs.getSize(); }

Dog Repository::getDog(int index) const { return this->dogs[index]; }

int Repository::findDog(const Dog &dog) const {
  /*
  Use the overloaded equality operator to find the index of a dog in the
  repository. If the dog is not found, return -1
  */

  for (int i = 0; i < this->dogs.getSize(); i++) {
    if (this->dogs[i] == dog) {
      return i;
    }
  }

  return -1;
}