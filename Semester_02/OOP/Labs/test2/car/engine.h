#pragma once
#include <string>

class Engine {
 protected:
  double base_price;

 public:
  Engine(double base_price = 3000) : base_price(base_price){};
  virtual ~Engine() = 0;
  virtual double getPrice() { return base_price; }
  virtual std::string toString() = 0;
};

class ElectricEngine : public Engine {
 private:
  int autonomy;

 public:
  ElectricEngine(int autonomy = 0) : Engine{}, autonomy(autonomy){};
  double getPrice() override;
  std::string toString() override;
};

class TurboEngine : public Engine {
 private:
 public:
  TurboEngine() : Engine{} {};
  double getPrice() override;
  std::string toString() override;
};