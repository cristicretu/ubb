#include "domain.h"

double Car::getPrice() {
  return engine->getPrice() + body_style == SEDAN ? 8000 : 1000;
}

std::string Car::toString() {
  return (static_cast<int>(body_style) == 0 ? "SEDAN" : "CONVERTIBLE") +
         engine->toString();
}