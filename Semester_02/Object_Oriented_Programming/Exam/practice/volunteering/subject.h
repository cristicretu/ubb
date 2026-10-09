#pragma once

#include <memory>
#include <vector>

#include "observer.h"

class Subject {
 private:
  std::vector<std::unique_ptr<Observer>> observers;

 public:
  void registerObserver(Observer *observer) {
    observers.emplace_back(observer);
  }

  void notify() {
    for (auto &observer : observers) {
      observer->update();
    }
  }
};
