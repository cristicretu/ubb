#pragma once
#include <fstream>
#include <sstream>

#include "domain.h"

class Repository {
 private:
  vector<Biologist> biologists;
  vector<Bacterium> bacteria;

 public:
  Repository() {
    load_biologists();
    load_bacteria();
  };
  ~Repository() {
    save_biologists();
    save_bacteria();
  };

  void load_biologists() {
    ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/microbial_world/"
        "biologists.txt");
    string name;
    vector<string> species;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      string speciesstring;
      getline(iss, speciesstring, '|');

      istringstream iss2(speciesstring);

      string speciesname;
      while (getline(iss2, speciesname, ',')) {
        species.push_back(speciesname);
      }

      Biologist b(name, species);
      biologists.emplace_back(b);
      species.clear();
    }
  }
  void load_bacteria() {
    ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/microbial_world/"
        "bacteriums.txt");
    string name, spacies;
    int size;
    vector<string> diseases;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, spacies, '|');
      string sizestring;
      getline(iss, sizestring, '|');
      size = stoi(sizestring);
      string diseasesstring;
      getline(iss, diseasesstring, '|');

      istringstream iss2(diseasesstring);

      string diseasename;
      while (getline(iss2, diseasename, ',')) {
        diseases.push_back(diseasename);
      }

      Bacterium b(name, spacies, size, diseases);
      bacteria.emplace_back(b);
      diseases.clear();
    }
  }

  void save_biologists() {
    ofstream fout(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/microbial_world/"
        "biologists.txt");
    for (auto &biologist : biologists) {
      fout << biologist.get_name() << "|";
      for (auto &species : biologist.get_species()) {
        fout << species << ",";
      }
      fout << endl;
    }
  }

  void save_bacteria() {
    ofstream fout(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/microbial_world/"
        "bacteriums.txt");
    for (auto &bacterium : bacteria) {
      fout << bacterium.get_name() << "|" << bacterium.get_spacies() << "|"
           << bacterium.get_size() << "|";
      for (auto &disease : bacterium.get_diseases()) {
        fout << disease << ",";
      }
      fout << endl;
    }
  }

  vector<Biologist> &get_biologists() { return biologists; }
  vector<Bacterium> &get_bacteria() { return bacteria; }
};
