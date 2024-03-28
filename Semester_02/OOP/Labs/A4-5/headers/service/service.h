#pragma once

#include "../domain/domain.h"
#include "../domain/dyn_array.h"
#include "../repository/dog_type_repository.h"
#include "../repository/repository.h"

class Service {
 private:
  Repository repository;
  DogTypeRepository dogTypeRepository;
  DynArray<Dog> adoptedDogs;

 public:
  Service();
  ~Service();

  // Admin mode
  void addDog(const std::string &breed, const std::string &name, int age,
              const std::string &photograph);
  void removeDog(int index);
  void updateDog(int index, const std::string &breed, const std::string &name,
                 int age, const std::string &photograph);
  DynArray<Dog> getDogs() const;
  int getNumberOfDogs() const;
  Dog getDog(int index) const;
  int findDog(const std::string &breed, const std::string &name, int age,
              const std::string &photograph) const;

  void generateNRandomDogs(int n);

  DynArray<Dog> getAdoptedDogs() const;
  void adoptDog(int index);
  DynArray<Dog> filterDogs(const std::string &breed, int age) const;

  // User mode
};