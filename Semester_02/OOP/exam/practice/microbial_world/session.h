#pragma once
#include <set>

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository repo;

 public:
  Session(){};

  vector<Biologist> get_biologists() { return repo.get_biologists(); };
  void add_bacterium(string name, string spacies, int size,
                     vector<string> diseases) {
    if (name.empty() || spacies.empty() || size <= 0 || diseases.empty()) {
      throw runtime_error("Invalid bacterium");
    }
    if (repo.find_bacterium(name) == -1) {
      repo.add_bacterium(name, spacies, size, diseases);
      notify();
    } else {
      throw runtime_error("Bacterium already exists");
    }
  };

  vector<Bacterium> get_bacteria_by_biologist(string name) {
    return repo.get_bacteria_by_biologist(name);
  }

  Bacterium get_bacterium_by_name(string name) {
    return repo.get_bacterium_by_name(name);
  }

  vector<string> get_species_by_biologist(string name) {
    auto bacteria = repo.get_bacteria_by_biologist(name);

    set<string> species;
    for (const auto &bacterium : bacteria) {
      species.insert(bacterium.get_spacies());
    }

    return vector<string>(species.begin(), species.end());
  }
};
