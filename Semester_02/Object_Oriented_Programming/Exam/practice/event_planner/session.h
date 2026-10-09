#pragma once
#include <algorithm>
#include <map>

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;
  vector<pair<Event, Person>> going;

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

  void markPersonAsGoing(Event ev, Person p) {
    going.emplace_back(ev, p);
    notify();
  }

  vector<Event> mostPopularEvents() {
    vector<Event> ev = repo.getEvents();

    vector<pair<Event, int>> popularity;

    for (auto x : ev) {
      int count = 0;
      for (auto y : going) {
        if (y.first.getName() == x.getName() &&
            y.first.getLatitude() == x.getLatitude() &&
            y.first.getLongitude() == x.getLongitude() &&
            y.first.getDate() == x.getDate()) {
          count++;
        }
      }
      popularity.emplace_back(x, count);
    }

    sort(popularity.begin(), popularity.end(),
         [](const pair<Event, int> &a, const pair<Event, int> &b) {
           return a.second > b.second;
         });

    vector<Event> res;

    for (auto x : popularity) {
      res.emplace_back(x.first);
    }

    return res;
  }

  bool isPersonGoing(Event ev, Person p) {
    for (auto x : going) {
      if (x.first.getName() == ev.getName() &&
          x.first.getLatitude() == ev.getLatitude() &&
          x.first.getLongitude() == ev.getLongitude() &&
          x.first.getDate() == ev.getDate() &&
          x.second.getName() == p.getName() &&
          x.second.getLatitude() == p.getLatitude() &&
          x.second.getLongitude() == p.getLongitude()) {
        return true;
      }
    }
  }
};
