#pragma once
#include <algorithm>

#include "domain.h"

class Repository {
 private:
  vector<Astronomer> astronomeers;
  vector<Star> stars;

 public:
  Repository() {
    loadAstornomers();
    loadStars();
  };
  ~Repository() { saveStars(); }

  vector<Astronomer>& getAstronomers() { return astronomeers; }
  vector<Star>& getStars() { return stars; }

  void loadAstornomers() {
    ifstream fin("../astronomers.txt");
    string name, constellation, line;

    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, constellation);

      astronomeers.emplace_back(Astronomer(name, constellation));
    }
  }

  void loadStars() {
    ifstream fin("../stars.txt");
    string name, constellation, ra, dec, diameter, line;
    string line;

    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, constellation, '|');
      getline(iss, ra, '|');
      getline(iss, dec, '|');
      getline(iss, diameter);

      stars.emplace_back(
          Star(name, constellation, stoi(ra), stoi(dec), stod(diameter)));
    }
  }

  void saveStars() {
    ofstream fout("../stars.txt");

    sort(stars.begin(), stars.end(), [](const Star& a, const Star& b) {
      return a.getConstellation() < b.getConstellation();
    });

    for (const auto& star : stars) {
      fout << star.getName() << "|" << star.getConstellation() << "|"
           << star.getRa() << "|" << star.getDec() << "|" << star.getDiameter()
           << endl;
    }
  }
};
