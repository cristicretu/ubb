#pragma once

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;

 public:
  Session(Repository &repo) : repo(repo){};

  vector<Driver> getDrivers() { return repo.getDrivers(); }
  vector<Report> getReports() { return repo.getReports(); }

  void addReport(string description, string name, int lat, int lg, bool score) {
    Driver drv = repo.getDriverByName(name);
    if (drv.getName() == "") {
      throw runtime_error("Driver does not exist");
    }
    auto dist = sqrt(pow(drv.getLat() - lat, 2) + pow(drv.getLg() - lg, 2));
    if (description.empty() || dist > 20) {
      throw runtime_error("Invalid report");
    }
    repo.addReport(Report(description, name, lat, lg, score));

    notify();
  }

  vector<Report> getInterestReports(string name) {
    vector<Report> ans;
    Driver drv = repo.getDriverByName(name);

    for (auto x : repo.getReports()) {
      auto dist = sqrt(pow(drv.getLat() - x.getLat(), 2) +
                       pow(drv.getLg() - x.getLat(), 2));
      if (dist < 10) {
        ans.emplace_back(x);
      }
    }

    return ans;
  }
};
