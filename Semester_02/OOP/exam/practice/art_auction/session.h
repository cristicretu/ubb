#pragma once
#include <algorithm>

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository& repo;

 public:
  Session(Repository& repo) : repo(repo){};

  vector<User> getUsers() const { return repo.getUsers(); }
  vector<Item> getItems() const { return repo.getItems(); }
};
