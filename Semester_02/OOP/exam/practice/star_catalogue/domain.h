#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

class Astronomer {
 private:
  string name, constellation;

 public:
  Astronomer(string name, string constellation)
      : name(name), constellation(constellation){};

  string getName() const { return name; }
  string getConstellation() const { return constellation; }
};

class Star {
 private:
  string name, constellation;
  int ra, dec;
  double diameter;

 public:
  Star(string name, string constellation, int ra, int dec, double diameter)
      : name(name),
        constellation(constellation),
        ra(ra),
        dec(dec),
        diameter(diameter){};
  string getName() const { return name; }
  string getConstellation() const { return constellation; }
  int getRa() const { return ra; }
  int detDec() const { return dec; }
  double getDiameter() const { return diameter; }
};
