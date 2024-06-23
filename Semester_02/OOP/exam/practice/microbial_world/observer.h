#pragma once

class Observer {
 public:
  virtual ~Observer() = default;
  virtual void update() const = 0;
};
