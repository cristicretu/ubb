#pragma once
#include <iostream>
#include <string>
using namespace std;

class Event {
 private:
  string organiser, name, description, date;
  int latitude, longitude;

 public:
  Event(string organiser, string name, string description, int latitude,
        int longitude, string date)
      : organiser(organiser),
        name(name),
        description(description),
        latitude(latitude),
        longitude(longitude){};

  int getLatitude() const { return latitude; }
  int getLongitude() const { return longitude; }
  string getDate() const { return date; }
  string getOrganiser() const { return organiser; }
  string getName() const { return name; }
  string getDescription() const { return description; }
};

class Person {
 private:
  string name;
  bool status;
  int latitude, longitude;

 public:
  Person(string name, int latitude, int longitude, bool status = false)
      : name(name), latitude(latitude), longitude(longitude), status(status){};

  string getName() const { return name; }
  bool getStatus() const { return status; }
  int getLatitude() const { return latitude; }
  int getLongitude() const { return longitude; }
};