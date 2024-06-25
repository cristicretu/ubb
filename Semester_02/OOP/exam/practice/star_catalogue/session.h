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

  void addStar(const string &name, const string &constellation, const int ra,
               const int dec, const double diameter) {
    if (ra < 0 || ra > 24 || dec < -90 || dec > 90 || diameter < 0) {
      throw invalid_argument("Invalid star data");
    }
    auto st = Star(name, constellation, ra, dec, diameter);
    repo.addStar(st);
    notify();
  }
};
