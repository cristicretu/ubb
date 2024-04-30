#pragma once
#include "repository.h"

class FileRepository : public Repository {
 private:
  std::string filename;

 public:
  FileRepository(const std::string &fileName) : filename{fileName} {
    loadDogs();
  };
  virtual ~FileRepository();
  void loadDogs();
  void saveDogs();

  void addDog(const Dog &dog);
  void removeDog(int index);
  void updateDog(int index, const Dog &dog);
};