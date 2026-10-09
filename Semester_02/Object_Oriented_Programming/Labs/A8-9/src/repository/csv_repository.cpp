#include "../../headers/repository/csv_repository.h"

#include <fstream>

CSVRepository::~CSVRepository() { saveDogs(); }

void CSVRepository::saveDogs() {
  std::ofstream file(this->filename);

  for (const auto &dog : this->getDogs()) {
    file << dog.getBreed() << "," << dog.getName() << "," << dog.getAge() << ","
         << dog.getPhotograph() << "\n";
  }

  file.close();
}

void CSVRepository::addDog(const Dog &dog) {
  Repository::addDog(dog);
  saveDogs();
}

void CSVRepository::removeDog(int index) {
  Repository::removeDog(index);
  saveDogs();
}

void CSVRepository::updateDog(int index, const Dog &dog) {
  Repository::updateDog(index, dog);
  saveDogs();
}
