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
  void loadPersons();
  void saveEvents() {
    ofstream fout("../events.txt");

    for (auto x : events) {
      fout << x.getOrganiser() << "|" << x.getName() << "|"
           << x.getDescription() << "|" << x.getLatitude() << "|"
           << x.getLongitude() << "|" << x.getDate() << '\n';
    }
    fout.close();
  }
  void savePersons();

  vector<Event> &getEvents() { return events; }
  vector<Person> &getPersons() { return persons; }
};
