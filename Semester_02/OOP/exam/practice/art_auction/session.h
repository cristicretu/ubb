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

  void addItem(string name, string category, int price) {
    if (name.empty() || price < 1) {
      throw runtime_error("Invalid item");
    }

    auto it = Item(name, category, price);

    repo.addItem(it);

    notify();
  }
};
