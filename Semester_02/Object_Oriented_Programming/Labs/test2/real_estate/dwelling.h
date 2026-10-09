#pragma once
#include <string>

class Dwelling {
 private:
  double price;
  bool isProfitable;

 public:
  Dwelling(double price = 0, bool isProfitable = false)
      : price(price), isProfitable(isProfitable) {}
  ~Dwelling();

  double getPrice() const { return price; }
  bool isDwellingProfitable() const { return isProfitable; }
  std::string toString() const {
    return "Price: " + std::to_string(price) +
           ", Profitable: " + (isProfitable ? "Yes" : "No");
  }
};
