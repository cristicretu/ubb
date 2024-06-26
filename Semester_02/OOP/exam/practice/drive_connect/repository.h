#pragma once
#include "domain.h"

class Repository {
 private:
  vector<Driver> drivers;
  vector<Report> reports;

 public:
  Repository() {
    loadDrivers();
    loadReports();
  };
  ~Repository() { saveReports(); };

  void loadDrivers() {
    ifstream fin("../drivers.txt");
    string name, lat, lg, score;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, lat, '|');
      getline(iss, lg, '|');
      getline(iss, score);

      drivers.emplace_back(Driver(name, stoi(lat), stoi(lg), stoi(score)));
    }
    fin.close();
  }

  void loadReports() {
    ifstream fin("../reports.txt");
    string description, reporter, lat, lg, status;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, description, '|');
      getline(iss, reporter, '|');
      getline(iss, lat, '|');
      getline(iss, lg, '|');
      getline(iss, status);

      reports.emplace_back(
          Report(description, reporter, stoi(lat), stoi(lg), stoi(status)));
    }
    fin.close();
  }

  void saveReports() {
    ofstream fout("../reports.txt");
    for (const auto &r : reports) {
      fout << r.getDescription() << "|" << r.getReporter() << "|" << r.getLat()
           << "|" << r.getLg() << "|" << r.getStatus() << '\n';
    }
    fout.close();
  }
};