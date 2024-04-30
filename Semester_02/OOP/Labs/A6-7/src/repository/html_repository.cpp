#include "../../headers/repository/html_repository.h"

#include <fstream>

HTMLRepository::~HTMLRepository() { saveDogs(); }

void HTMLRepository::saveDogs() {
  std::ofstream file(this->filename);

  file << "<!DOCTYPE "
          "html>\n<html>\n<head>\n<title>Dogs</title>\n</head>\n<body>\n<table "
          "border=\"1\">\n<tr>\n<td>Breed</td>\n<td>Name</td>\n<td>Age</"
          "td>\n<td>Photograph</td>\n</tr>\n";

  for (const auto &dog : this->getDogs()) {
    file << "<tr>\n<td>" << dog.getBreed() << "</td>\n<td>" << dog.getName()
         << "</td>\n<td>" << dog.getAge() << "</td>\n<td><a href=\""
         << dog.getPhotograph() << "\">Link</a></td>\n</tr>\n";
  }

  file << "</table>\n</body>\n</html>";

  file.close();
}

void HTMLRepository::addDog(const Dog &dog) {
  Repository::addDog(dog);
  saveDogs();
}

void HTMLRepository::removeDog(int index) {
  Repository::removeDog(index);
  saveDogs();
}

void HTMLRepository::updateDog(int index, const Dog &dog) {
  Repository::updateDog(index, dog);
  saveDogs();
}