#pragma once
#include "repository.h"

class CSVRepository : public Repository {
 private:
  std::string filename;

 public:
  CSVRepository(const std::string &fileName) : filename{fileName} {};
  virtual ~CSVRepository();
  void saveDogs();

  void addDog(const Dog &dog);
  void removeDog(int index);
  void updateDog(int index, const Dog &dog);
};