#pragma once

#include <set>

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;
  vector<string> messages;

  set<pair<string, string>> validations;

 public:
  Session(Repository &repo) : repo(repo){};

  vector<Driver> getDrivers() { return repo.getDrivers(); }
  vector<Report> getReports() { return repo.getReports(); }
  vector<string> getMessages() { return messages; }

  void addMessage(string name, string msg) {
    Driver drv = repo.getDriverByName(name);
    if (drv.getName() == "") {
      throw runtime_error("Driver does not exist");
    }
    if (msg.empty()) {
      throw runtime_error("Invalid message");
    }
    messages.emplace_back("[" + name + "]" + ": " + msg);

    notify();
  }

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

  void addValidation(string descr, string validator) {
    auto rep = repo.getReportByDescription(descr);
    if (rep.getReporter() == validator) {
      throw runtime_error("Cannot validate own report");
    }

    validations.insert({descr, validator});

    int numberOfValidations = 0;

    for (auto x : validations) {
      if (x.first == descr) {
        numberOfValidations++;
      }
    }

    if (numberOfValidations >= 2) {
      repo.setReportStatus(descr, true);
      repo.increaseDriverScore(rep.getReporter());
    }

    notify();
  }
};
