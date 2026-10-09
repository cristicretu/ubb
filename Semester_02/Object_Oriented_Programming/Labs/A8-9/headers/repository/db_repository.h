#pragma once
#include <sqlite3.h>

#include "repository.h"

class DBRepository : public Repository {
 private:
  sqlite3 *db;
  std::string dbName;

 public:
  DBRepository(const std::string &dbName) : dbName{dbName} { loadDogs(); }
  virtual ~DBRepository();

  void loadDogs();
  void saveDogs();

  void addDog(const Dog &dog);
  void removeDog(int index);
  void updateDog(int index, const Dog &dog);
};