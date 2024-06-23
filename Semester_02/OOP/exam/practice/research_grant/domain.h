#pragma once
#include <iostream>
#include <string>

using namespace std;

class Researcher {
 private:
  string name, position;  // pos can be "pdh, postdoc, senior"

 public:
  Researcher(string name, string position) : name(name), position(position){};
};

class Idea {
 private:
  string title, description, creator;
  bool status;   // accepted or not
  int duration;  // 1, 2,3
 public:
  Idea(string title, string description, string creator, bool status,
       int duration)
      : title(title),
        description(description),
        creator(creator),
        status(status),
        duration(duration){};
};