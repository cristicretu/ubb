#pragma once
#include <string>

class Task {
 private:
  std::string description;
  int duration, priority;

 public:
  Task(std::string description, int duration, int priority)
      : description(description), duration(duration), priority(priority){};
  ~Task(){};

  std::string getDescription() const { return description; }

  int getDuration() const { return duration; }

  int getPriority() const { return priority; }

  std::string to_string() {
    return description + " | " + std::to_string(duration) + " | " +
           std::to_string(priority);
  }
};
