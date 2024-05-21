#pragma once

#include "repository.h"

class HTMLRepository : public Repository {
 private:
  std::string filename;

 public:
  HTMLRepository(const std::string &fileName) : filename{fileName} {};
  virtual ~HTMLRepository();
  void saveDogs();

  void addDog(const Dog &dog);
  void removeDog(int index);
  void updateDog(int index, const Dog &dog);
};