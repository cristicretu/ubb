#include "domain.h"

Building::~Building() {}

bool Block::mustBeRestored() {
  return constructionYear < 1970 && occupiedApartments / totalApartments > 0.8;
}

std::string Block::toString() {
  return "Block " + address + " was built in " +
         std::to_string(constructionYear) + " and has " +
         std::to_string(totalApartments) + " apartments, out of which " +
         std::to_string(occupiedApartments) + " are occupied.";
}

bool House::mustBeRestored() { return constructionYear < 1900; }

std::string House::toString() {
  return "House " + address + " was built in " +
         std::to_string(constructionYear) + " and is " +
         (isHistorical ? "" : "not ") + "historical.";
}
