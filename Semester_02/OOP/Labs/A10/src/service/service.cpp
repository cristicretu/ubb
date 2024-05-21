#include "../../headers/service/service.h"

#include <algorithm>
#include <iostream>

#include "../../headers/domain/domain.h"

Service::Service() {
  this->repository = nullptr;
  this->adoptedRepository = nullptr;
  this->dogTypeRepository = DogTypeRepository();

  this->undoActions = std::stack<std::shared_ptr<Action>>();
  this->redoActions = std::stack<std::shared_ptr<Action>>();
}

Service::~Service() {
  if (this->repository != nullptr) {
    delete this->repository;
  }
  if (this->adoptedRepository != nullptr) {
    delete this->adoptedRepository;
  }
}

/// @brief If the dog already exists, throw an exception, otherwise add the
/// dog
/// @param breed The breed of the dog
/// @param name The name of the dog
/// @param age The age of the dog
/// @param photograph The photograph of the dog
void Service::addDog(const std::string &breed, const std::string &name, int age,
                     const std::string &photograph) {
  Dog dog(breed, name, age, photograph);

  int index = this->findDog(breed, name, age, photograph);
  if (index != -1) {
    throw std::invalid_argument("Dog already exists!");
  }

  (*this->repository).addDog(dog);

  std::shared_ptr<Action> action = std::make_shared<ActionAdd>(
      *this->repository, Dog(breed, name, age, photograph));

  this->undoActions.push(action);
  while (!this->redoActions.empty()) {
    this->redoActions.pop();
  }
}

/// @brief Remove a dog from the repository, if it exists
/// @param index The index of the dog to be removed
void Service::removeDog(int index) {
  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist");
  }
  auto dog = (*this->repository).getDog(index);

  (*this->repository).removeDog(index);

  std::shared_ptr<Action> action =
      std::make_shared<ActionRemove>(*this->repository, dog);

  this->undoActions.push(action);
  while (!this->redoActions.empty()) {
    this->redoActions.pop();
  }
}

/// @brief  Update a dog from the repository, if it exists
/// @param index The index of the dog to be updated
/// @param breed The new breed of the dog
/// @param name The new name of the dog
/// @param age  The new age of the dog
/// @param photograph The new photograph of the dog
void Service::updateDog(int index, const std::string &breed,
                        const std::string &name, int age,
                        const std::string &photograph) {
  Dog dog(breed, name, age, photograph);

  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist!");
  }

  auto oldDog = (*this->repository).getDog(index);

  (*this->repository).updateDog(index, dog);

  std::shared_ptr<Action> action =
      std::make_shared<ActionUpdate>(*this->repository, oldDog, dog);

  this->undoActions.push(action);
  while (!this->redoActions.empty()) {
    this->redoActions.pop();
  }
}

std::vector<Dog> Service::getDogs() const {
  return (*this->repository).getDogs();
}

int Service::getNumberOfDogs() const {
  return (*this->repository).getNumberOfDogs();
}

Dog Service::getDog(int index) const {
  return (*this->repository).getDog(index);
}

int Service::findDog(const std::string &breed, const std::string &name, int age,
                     const std::string &photograph) const {
  Dog dog(breed, name, age, photograph);

  return (*this->repository).findDog(dog);
}

void Service::generateNRandomDogs(int n = 10) {
  /*
  Use the dog type repository to generate n random dogs (with reald nmeas and
  breeds) If the API call fails, use a default image
  */
  for (int i = 0; i < n; i++) {
    std::string breed = this->dogTypeRepository.getRandomBreed();
    std::string name = this->dogTypeRepository.getRandomName();
    int age = rand() % 15 + 1;
    std::pair<std::string, bool> res =
        this->dogTypeRepository.getDogImageUrl(breed);
    std::string photograph = res.first == ""
                                 ? "https://images.dog.ceo/breeds/hound-basset/"
                                   "n02088238_10473.jpg"  /// fallback image
                                 : res.first;
    this->addDog(breed, name, age, photograph);
  }
}

std::vector<Dog> Service::getAdoptedDogs() const {
  return (*this->adoptedRepository).getDogs();
}

/// @brief Add a dog to the list of adopted dogs and remove it from the list
/// of available dogs
/// @param index The index of the dog to be adopted, if it exists
void Service::adoptDog(int index) {
  if (index < 0 || index >= this->getNumberOfDogs()) {
    throw std::invalid_argument("Dog does not exist!");
  }

  Dog dog = (*this->repository).getDog(index);
  (*this->adoptedRepository).addDog(dog);
  (*this->repository).removeDog(index);
}

/// @brief Filter the dogs by breed and age
/// @param breed The breed to filter by
/// @param age  The age to filter by
/// @return   A list of dogs that match the criteria
std::vector<Dog> Service::filterDogs(const std::string &breed, int age) const {
  std::vector<Dog> dogs = (*this->repository).getDogs();
  std::vector<Dog> filteredDogs = std::vector<Dog>(dogs.size());

  auto it = std::copy_if(
      dogs.begin(), dogs.end(), filteredDogs.begin(), [breed, age](Dog dog) {
        return (breed.size() == 0) ||
               (dog.getBreed() == breed && dog.getAge() < age);
      });

  filteredDogs.resize(std::distance(filteredDogs.begin(), it));

  return filteredDogs;
}