#pragma once
#include <vector>

#include "domain.h"

class Repository {
 private:
  std::vector<Gene> data;

 public:
  Repository();
  ~Repository();
  bool addGene(Gene gene);
  bool removeGene(Gene gene);
  int indexOfGene(Gene gene);
  std::vector<Gene> getGenes() const;
};