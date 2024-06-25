#pragma once
#include <algorithm>

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;

 public:
  Session(Repository &repo) : repo(repo){};

  vector<Event> getEvents() {
    auto ev = repo.getEvents();
    sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) {
      return a.getDate() < b.getDate();
    });
    return ev;
  }

  vector<Person> getPersons() { return repo.getPersons(); }

  void addEvent(string organiser, string name, string description, int latitude,
                int longitude, string date) {
    auto ev = Event(organiser, name, description, latitude, longitude, date);
    int pos = repo.getEvent(ev);

    if (pos == -1) {
      repo.addEvent(ev);
      notify();
    } else {
      throw runtime_error("Event already exists");
    }
  }
};
