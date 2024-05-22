#pragma once
#include <string>

class Item {
 private:
  std::string category, name;
  int quantity;

 public:
  Item() : category(""), name(""), quantity(0) {}

  Item(std::string category, std::string name, int quantity)
      : category{category}, name{name}, quantity{quantity} {};

  std::string getCategory() const { return category; }
  std::string getName() const { return name; }
  int getQuantity() const { return quantity; }

  std::string to_string() {
    return category + " | " + name + " | " + std::to_string(quantity);
  }
};
