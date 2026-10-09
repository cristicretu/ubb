#pragma once
#include <fstream>
#include <sstream>
#include <vector>

#include "domain.h"

class Repository {
 private:
  vector<Event> events;
  vector<Person> persons;

 public:
  Repository() {
    loadEvents();
    loadPersons();
  };
  ~Repository() {
    saveEvents();
    savePersons();
  }

  void addEvent(Event &ev) { events.emplace_back(ev); }

  int getEvent(Event &ev) {
    for (int i = 0, n = events.size(); i < n; ++i) {
      if (events[i].getName() == ev.getName() &&
          events[i].getLatitude() == ev.getLatitude() &&
          events[i].getLongitude() == ev.getLongitude()) {
        return i;
      }
    }
    return -1;
  }

  void loadEvents() {
    string organiser, name, description, date;
    string latitude, longitude;

    string line;
    ifstream fin("../events.txt");
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, organiser, '|');
      getline(iss, name, '|');
      getline(iss, description, '|');
      getline(iss, latitude, '|');
      getline(iss, longitude, '|');
      getline(iss, date);

      auto ev = Event(organiser, name, description, stoi(latitude),
                      stoi(longitude), date);

      events.emplace_back(ev);
    }
    fin.close();
  };
  void loadPersons() {
    string name, latitude, longitude, status;
    string line;

    ifstream fin("../persons.txt");
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, latitude, '|');
      getline(iss, longitude, '|');
      getline(iss, status);

      auto p = Person(name, stoi(latitude), stoi(longitude), stoi(status));
      persons.emplace_back(p);
    }
    fin.close();
  }
  void saveEvents() {
    ofstream fout("../events.txt");

    for (auto x : events) {
      fout << x.getOrganiser() << "|" << x.getName() << "|"
           << x.getDescription() << "|" << x.getLatitude() << "|"
           << x.getLongitude() << "|" << x.getDate() << '\n';
    }
    fout.close();
  }
  void savePersons() {
    ofstream fout("../persons.txt");

    for (auto x : persons) {
      fout << x.getName() << "|" << x.getLatitude() << "|" << x.getLongitude()
           << "|" << x.getStatus() << '\n';
    }
    fout.close();
  }
  vector<Event> &getEvents() { return events; }
  vector<Person> &getPersons() { return persons; }
};
