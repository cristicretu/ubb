#pragma once
#include "../domain/domain.h"

class AdoptedList {
 private:
  std::vector<Dog> dogs;

 public:
  AdoptedList();
  ~AdoptedList();

  void addDog(const Dog &dog);
  void removeDog(int index);
  std::vector<Dog> getDogs() const;
  int findDog(const Dog &dog) const;
};