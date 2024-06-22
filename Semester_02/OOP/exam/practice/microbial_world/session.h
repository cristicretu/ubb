#pragma once
#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository repo;

 public:
  Session(){};

  vector<Biologist> get_biologists() { return repo.get_biologists(); };
};
