#include "repository.h"

Repository::Repository() { data = std::vector<Gene>(); }

Repository::~Repository() {}

bool Repository::addGene(Gene gene) {
  if (indexOfGene(gene) == -1) {
    data.push_back(gene);
    return true;
  }
  return false;
}

bool Repository::removeGene(Gene gene) {
  int index = indexOfGene(gene);
  if (index != -1) {
    data.erase(data.begin() + index);
    return true;
  }
  return false;
}

int Repository::indexOfGene(Gene gene) {
  for (int i = 0; i < data.size(); i++) {
    if (data[i] == gene) {
      return i;
    }
  }
  return -1;
}

std::vector<Gene> Repository::getGenes() const { return data; }