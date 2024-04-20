#include "engine.h"

Engine::~Engine() {}

std::string Engine::toString() {
  return "Base: Price - " + std::to_string(this->base_price) + "\n";
}

double ElectricEngine::getPrice() {
  return Engine::getPrice() + double(this->autonomy) * 0.1;
}

std::string ElectricEngine::toString() {
  return "Engine type - Electric\nElectric: Price - " +
         std::to_string(this->getPrice()) + "\n";
}

double TurboEngine::getPrice() { return Engine::getPrice() + 100; }

std::string TurboEngine::toString() {
  return "Engine type - Turbo\nTurbo: Price - " +
         std::to_string(this->getPrice()) + "\n";
}
