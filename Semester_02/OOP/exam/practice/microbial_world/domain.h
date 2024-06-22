#pragma once
#include <string>
#include <vector>

using namespace std;

class Biologist {
 private:
  string name;
  vector<string> species;

 public:
  Biologist(string name, vector<string> species)
      : name(name), species(species){};
};

class Bacterium {
 private:
  string name, spacies;
  int size;
  vector<string> diseases;

 public:
  Bacterium(string name, string spacies, int size, vector<string> diseases)
      : name(name), spacies(spacies), size(size), diseases(diseases){};
};
