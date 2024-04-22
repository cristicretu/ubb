#pragma once
#include <fstream>
#include <vector>

#include "domain.h"

class Service {
 private:
  std::vector<Building*> buildings;

 public:
  Service() {
    buildings.push_back(new Block("B1", 1960, 100, 90));
    buildings.push_back(new Block("B2", 1980, 50, 5));
    buildings.push_back(new House("H1", 1900, true));
    buildings.push_back(new House("H2", 2000, false));
  };
  ~Service() {
    for (auto b : buildings) {
      delete b;
    }
  };
  std::vector<Building*> getBuildings() { return buildings; }
  std::vector<Building*> getBuildingsToRestore(int year);
  void writeToFile(std::string filename);
};
