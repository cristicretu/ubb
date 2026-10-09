#pragma once

#include <memory>
#include <vector>

#include "observer.h"

class Subject {
 private:
  std::vector<Observer*> observers;

 public:
  Subject() = default;
  ~Subject() = default;
  void registerObserver(Observer* observer) {
    observers.emplace_back(observer);
  }

  void notify() {
    for (auto& observer : observers) {
      observer->update();
    }
  }
};
