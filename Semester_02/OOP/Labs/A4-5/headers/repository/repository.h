#pragma once
#include <iostream>

#include "../domain/domain.h"
#include "../domain/dyn_array.h"

class Repository {
 private:
  DynArray<Dog> dogs;

 public:
  Repository();
  ~Repository();

  // Admin mode
  void addDog(const Dog &dog);
  void removeDog(int index);
  void updateDog(int index, const Dog &dog);
  DynArray<Dog> getDogs() const;
  int getNumberOfDogs() const;
  Dog getDog(int index) const;
  int findDog(const Dog &dog) const;

  // User mode
};