#pragma once

class Observer {
 public:
  virtual ~Observer() = default;
  Observer() = default;
  virtual void update() const = 0;
};
