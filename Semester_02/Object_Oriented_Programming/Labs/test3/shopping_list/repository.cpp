#include "repository.h"

#include <fstream>
#include <sstream>

void Repository::loadData() {
  std::ifstream fin(filename);

  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);
    std::string category, name, quantity;
    std::getline(iss, category, '|');
    std::getline(iss, name, '|');
    std::getline(iss, quantity, '|');
    int quantityValue = std::stoi(quantity);
    items.push_back(Item(category, name, quantityValue));
  }
}

std::vector<Item> Repository::getItems() { return items; }