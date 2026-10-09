#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

class Researcher {
 private:
  string name, position;

 public:
  Researcher(string name, string position) : name(name), position(position){};

  string getName() const { return name; }
  string getPosition() const { return position; }
};

class Idea {
 private:
  string title, description, creator;
  bool status;
  int duration;

 public:
  Idea(string title, string description, bool status, string creator,
       int duration)
      : title(title),
        description(description),
        status(status),
        creator(creator),
        duration(duration){};

  string getTitle() const { return title; }
  string getDescription() const { return description; }
  bool getStatus() const { return status; }
  string getCreator() const { return creator; }
  int getDuration() const { return duration; }

  void setStatus(bool status) { this->status = status; }
  void setDescription(string desc) { this->description = desc; }
};
