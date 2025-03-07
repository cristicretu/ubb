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
  int getDec() const { return dec; }
  double getDiameter() const { return diameter; }

  void setName(string name) { this->name = name; }
  void setConstellation(string name) { this->constellation = name; }
  void setRa(int ra) { this->ra = ra; }
  void setDec(int dec) { this->dec = dec; }
  void setDiameter(double diameter) { this->diameter = diameter; }
};
