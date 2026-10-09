#include "service.h"

std::vector<Building*> Service::getBuildingsToRestore(int year) {
  std::vector<Building*> result;
  for (auto b : buildings) {
    if (b->mustBeRestored() && b->getConstructionYear() < year) {
      result.push_back(b);
    }
  }
  return result;
}

void Service::writeToFile(std::string filename) {
  std::ofstream out(filename);
  for (auto b : buildings) {
    out << b->toString() << std::endl;
  }
  out.close();
}