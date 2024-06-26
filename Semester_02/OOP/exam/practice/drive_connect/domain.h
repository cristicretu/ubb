#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

class Driver {
 private:
  string name;
  int lat, lg, score;

 public:
  Driver(string name, int lat, int lg, int score)
      : name(name), lat(lat), lg(lg), score(score){};
  ;

  string getName() const { return name; }
  int getLat() const { return lat; }
  int getLg() const { return lg; }
  int getScore() const { return score; }
};

class Report {
 private:
  string description, reporter;
  bool status;
  int lat, lg;

 public:
  Report(string description, string reported, int lat, int lg, bool status)
      : description(description),
        reporter(reported),
        lat(lat),
        lg(lg),
        status(status){};

  int getLat() const { return lat; }
  int getLg() const { return lg; }
  string getDescription() const { return description; }
  string getReporter() const { return reporter; }
  bool getStatus() const { return status; }
};