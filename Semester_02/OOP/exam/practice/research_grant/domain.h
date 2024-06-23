#pragma once
#include <iostream>
#include <string>

using namespace std;

class Researcher {
 private:
  string name, position;  // pos can be "pdh, postdoc, senior"

 public:
  Researcher(string name, string position) : name(name), position(position){};

  string getName() const { return name; }
  string getPosition() const { return position; }
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

  string getTitle() const { return title; }
  string getDescription() const { return description; }
  string getCreator() const { return creator; }
  bool getStatus() const { return status; }
  int getDuration() const { return duration; }

  void setStatus(bool status) { this->status = status; }
  void setTitle(string title) { this->title = title; }
  void setDescription(string description) { this->description = description; }
  void setCreator(string creator) { this->creator = creator; }
  void setDuration(int duration) { this->duration = duration; }
};