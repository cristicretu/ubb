#pragma once
#include <fstream>
#include <sstream>
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Weather> data;

 public:
  Repository() { loadData(); }
  ~Repository(){};

  void loadData();
  std::vector<Weather> getWeather() const;
};
