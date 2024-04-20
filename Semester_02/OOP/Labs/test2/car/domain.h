#pragma once
#include <string>

#include "engine.h"

class Car {
 private:
  enum bodyStyle { SEDAN, CONVERTIBLE };

  bodyStyle body_style;
  Engine* engine;

 public:
  Car(std::string body_style, Engine* engine) : engine(engine) {
    if (body_style == "SEDAN") {
      this->body_style = SEDAN;
    } else {
      this->body_style = CONVERTIBLE;
    }
  }

  ~Car() {}

  double getPrice();
  std::string toString();
};