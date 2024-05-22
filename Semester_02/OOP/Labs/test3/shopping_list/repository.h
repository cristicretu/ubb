#pragma once
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::string filename;
  std::vector<Item> items;

 public:
  Repository(std::string filename) : filename(filename) { loadData(); }
  ~Repository(){};

  void loadData();
  std::vector<Item> getItems();
};
