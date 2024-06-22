#pragma once
#include <algorithm>
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

  vector<Bacterium> get_bacteria_by_biologist(string name) {
    vector<Bacterium> result;
    vector<string> spacies;

    for (auto &biologist : biologists) {
      if (biologist.get_name() == name) {
        spacies = biologist.get_species();
        break;
      }
    }

    for (auto &bacterium : bacteria) {
      if (find(spacies.begin(), spacies.end(), bacterium.get_spacies()) !=
          spacies.end()) {
        result.emplace_back(bacterium);
      }
    }

    sort(result.begin(), result.end(), [](Bacterium &a, Bacterium &b) {
      return a.get_name() < b.get_name();
    });

    return result;
  }

  int find_bacterium(string name) {
    for (int i = 0; i < bacteria.size(); i++) {
      if (bacteria[i].get_name() == name) {
        return i;
      }
    }
    return -1;
  }

  void add_bacterium(string name, string spacies, int size,
                     vector<string> diseases) {
    Bacterium b(name, spacies, size, diseases);
    bacteria.emplace_back(b);
  }

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
