#include "../../headers/repository/file_repository.h"

#include <fstream>
#include <iostream>
#include <sstream>

FileRepository::~FileRepository() { saveDogs(); }

void FileRepository::loadDogs() {
  std::ifstream file(filename);

  if (!file.is_open()) throw std::runtime_error("File could not be opened");

  std::cout << "File opened\n";

  if (file.peek() == std::ifstream::traits_type::eof()) {
    file.close();
    return;
  }

  while (file) {
    Dog dog;
    if (file) {
      if (file.peek() == std::ifstream::traits_type::eof()) break;
      file >> dog;

      // std::cout << dog << '\n';

      try {
        this->addDog(dog);
      } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
      }
    }
  }
}

void FileRepository::saveDogs() {
  std::ofstream file(filename);
  if (!file.is_open()) throw std::runtime_error("File could not be opened");

  for (const auto &dog : this->getDogs()) {
    file << dog;
  }

  file.close();
}

void FileRepository::addDog(const Dog &dog) {
  Repository::addDog(dog);
  saveDogs();
}

void FileRepository::removeDog(int index) {
  Repository::removeDog(index);
  saveDogs();
}

void FileRepository::updateDog(int index, const Dog &dog) {
  Repository::updateDog(index, dog);
  saveDogs();
}