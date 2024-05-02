#pragma once

#include "../domain/domain.h"
#include "../repository/dog_type_repository.h"
#include "../repository/file_repository.h"
#include "../repository/repository.h"

class Service {
 private:
  Repository *repository;
  Repository *adoptedRepository;
  DogTypeRepository dogTypeRepository;

 public:
  Service();
  ~Service();

  void addDog(const std::string &breed, const std::string &name, int age,
              const std::string &photograph);
  void removeDog(int index);
  void updateDog(int index, const std::string &breed, const std::string &name,
                 int age, const std::string &photograph);
  std::vector<Dog> getDogs() const;
  int getNumberOfDogs() const;
  Dog getDog(int index) const;
  int findDog(const std::string &breed, const std::string &name, int age,
              const std::string &photograph) const;

  void generateNRandomDogs(int n);

  std::vector<Dog> getAdoptedDogs() const;
  void adoptDog(int index);
  std::vector<Dog> filterDogs(const std::string &breed, int age) const;

  void setAdoptedRepository(Repository *adoptedRepository) {
    this->adoptedRepository = adoptedRepository;
  }

  void setRepository(Repository *repository) { this->repository = repository; }
  void openAdoptedFile(const std::string &filename) {
    std::string command = "open " + filename;
    system(command.c_str());
  }
};