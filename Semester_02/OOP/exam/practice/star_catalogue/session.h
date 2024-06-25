#pragma once
#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;

 public:
  Session(Repository &repo) : repo(repo){};

  vector<Astronomer> &getAstronomers() { return repo.getAstronomers(); }
  vector<Star> &getStars() { return repo.getStars(); }
};
