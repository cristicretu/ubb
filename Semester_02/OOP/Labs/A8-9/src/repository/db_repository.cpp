#include "../../headers/repository/db_repository.h"

DBRepository::~DBRepository() { sqlite3_close(this->db); }

void DBRepository::loadDogs() {
  char *errMsg = 0;
  std::string breed, name, photograph;
  int age;
  int rc = sqlite3_open(this->dbName.c_str(), &db);
  if (rc) {
    sqlite3_close(db);
    throw std::runtime_error("Can't open database");
  }

  std::string sql = "SELECT * FROM dogs;";
  sqlite3_stmt *stmt;
  rc = sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    sqlite3_close(db);
    throw std::runtime_error("Failed to execute statement");
  }

  while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    breed = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    age = sqlite3_column_int(stmt, 2);
    photograph = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
    auto dog = Dog(breed, name, age, photograph);
    this->addDog(dog);
  }
}

void DBRepository::saveDogs() {
  char *errMsg = 0;
  int rc = sqlite3_open(this->dbName.c_str(), &db);
  if (rc) {
    sqlite3_close(db);
    throw std::runtime_error("Can't open database");
  }

  sqlite3_exec(db, "DROP TABLE IF EXISTS dogs;", NULL, NULL, &errMsg);
  sqlite3_exec(db,
               "CREATE TABLE dogs (Breed TEXT, Name TEXT, Age INT, "
               "Photograph TEXT);",
               NULL, NULL, &errMsg);

  for (const auto &dog : this->getDogs()) {
    std::stringstream ss;
    ss << "INSERT INTO dogs (Breed, Name, Age, Photograph) VALUES ('"
       << dog.getBreed() << "', '" << dog.getName() << "', " << dog.getAge()
       << ", '" << dog.getPhotograph() << "');";
    sqlite3_exec(db, ss.str().c_str(), NULL, NULL, &errMsg);
  }
}

void DBRepository::addDog(const Dog &dog) {
  Repository::addDog(dog);
  this->saveDogs();
}

void DBRepository::removeDog(int index) {
  Repository::removeDog(index);
  this->saveDogs();
}

void DBRepository::updateDog(int index, const Dog &dog) {
  Repository::updateDog(index, dog);
  this->saveDogs();
}