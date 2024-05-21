#pragma once
#include <iostream>
#include <vector>

#include "../domain/domain.h"

class Repository {
 protected:
  std::vector<Dog> dogs;

 public:
  Repository();
  virtual ~Repository();

  virtual void addDog(const Dog &dog);
  virtual void removeDog(int index);
  virtual void updateDog(int index, const Dog &dog);
  std::vector<Dog> getDogs() const;
  int getNumberOfDogs() const;
  Dog getDog(int index) const;
  int findDog(const Dog &dog) const;
};