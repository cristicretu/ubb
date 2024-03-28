#include "../../headers/service/service.h"

#include <iostream>

#include "../../headers/domain/domain.h"

Service::Service() {
  this->repository = Repository();
  this->dogTypeRepository = DogTypeRepository();
  this->adoptedDogs = DynArray<Dog>();
}

Service::~Service() {}

void Service::addDog(const std::string &breed, const std::string &name, int age,
                     const std::string &photograph) {
  Dog dog(breed, name, age, photograph);

  int index = this->findDog(breed, name, age, photograph);
  if (index != -1) {
    throw std::invalid_argument("Dog already exists!");
  }

  this->repository.addDog(dog);
}

void Service::removeDog(int index) {
  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist");
  }

  this->repository.removeDog(index);
}

void Service::updateDog(int index, const std::string &breed,
                        const std::string &name, int age,
                        const std::string &photograph) {
  Dog dog(breed, name, age, photograph);

  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist!");
  }

  this->repository.updateDog(index, dog);
}

DynArray<Dog> Service::getDogs() const { return this->repository.getDogs(); }

int Service::getNumberOfDogs() const {
  return this->repository.getNumberOfDogs();
}

Dog Service::getDog(int index) const { return this->repository.getDog(index); }

int Service::findDog(const std::string &breed, const std::string &name, int age,
                     const std::string &photograph) const {
  Dog dog(breed, name, age, photograph);

  return this->repository.findDog(dog);
}

void Service::generateNRandomDogs(int n = 10) {
  /*
  Use the dog type repository to generate n random dogs (with reald nmeas and
  breeds) If the API call fails, use a default image
  */
  for (int i = 0; i < n; i++) {
    std::string breed = this->dogTypeRepository.getRandomBreed();
    std::string name = this->dogTypeRepository.getRandomName();
    int age = rand() % 20 + 1;
    std::pair<std::string, bool> res =
        this->dogTypeRepository.getDogImageUrl(breed);
    std::string photograph =
        res.first == ""
            ? "https://images.dog.ceo/breeds/hound-basset/n02088238_10473.jpg"
            : res.first;
    this->addDog(breed, name, age, photograph);
  }
}

DynArray<Dog> Service::getAdoptedDogs() const { return this->adoptedDogs; }

void Service::adoptDog(int index) {
  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist!");
  }

  Dog dog = this->repository.getDog(index);
  this->adoptedDogs.add(dog);
  this->repository.removeDog(index);
}

DynArray<Dog> Service::filterDogs(const std::string &breed, int age) const {
  DynArray<Dog> dogs = this->repository.getDogs();
  DynArray<Dog> filteredDogs = DynArray<Dog>();

  for (int i = 0; i < dogs.getSize(); i++) {
    Dog dog = dogs[i];
    if ((breed.size() == 0) ||
        (dog.getBreed() == breed && dog.getAge() < age)) {
      filteredDogs.add(dog);
    }
  }

  return filteredDogs;
}
