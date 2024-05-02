#include "../../headers/domain/DogValidator.h"

#include "../../headers/repository/dog_type_repository.h"

void DogValidator::validateDogName(const std::string &name) {
  if (name.size() < 3) {
    throw DogException(
        "The name of the dog must be at least 3 characters long");
  }
}

void DogValidator::validateDogBreed(const std::string &breed) {
  if (breed.size() < 3) {
    throw DogException(
        "The breed of the dog must be at least 3 characters long");
  }

  DogTypeRepository dg;

  if (dg.findBreed(breed) == -1) {
    throw DogException("The breed of the dog must be a valid breed");
  }
}

void DogValidator::validateDogYear(const int &year) {
  if (year < 0) {
    throw DogException("The year of the dog must be a positive integer");
  }

  if (year > 100) {
    throw DogException("The year of the dog must be less than 100");
  }
}
void DogValidator::validateDogPhoto(const std::string &photo) {
  if (photo.size() < 3) {
    throw DogException(
        "The photograph of the dog must be at least 3 characters long");
  }
}

void DogValidator::validateDogIdentifiers(const std::string &breed,
                                          const std::string &name,
                                          const int &year,
                                          const std::string &photo) {
  std::vector<std::invalid_argument> errors;
  try {
    validateDogName(name);
  } catch (DogException &e) {
    errors.push_back(std::invalid_argument(e.what()));
  }
  try {
    validateDogBreed(breed);
  } catch (DogException &e) {
    errors.push_back(std::invalid_argument(e.what()));
  }
  try {
    validateDogYear(year);
  } catch (DogException &e) {
    errors.push_back(std::invalid_argument(e.what()));
  }
  try {
    validateDogPhoto(photo);
  } catch (DogException &e) {
    errors.push_back(std::invalid_argument(e.what()));
  }
  if (errors.size() > 0) {
    throw DogException(errors);
  }
}