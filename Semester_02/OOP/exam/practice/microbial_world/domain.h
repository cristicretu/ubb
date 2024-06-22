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
  string get_name() { return name; };
  vector<string> get_species() { return species; };
};

class Bacterium {
 private:
  string name, spacies;
  int size;
  vector<string> diseases;

 public:
  Bacterium(string name, string spacies, int size, vector<string> diseases)
      : name(name), spacies(spacies), size(size), diseases(diseases){};

  string get_name() { return name; };
  string get_spacies() { return spacies; };
  int get_size() { return size; };
  vector<string> get_diseases() { return diseases; };
};
