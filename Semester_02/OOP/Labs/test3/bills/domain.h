#pragma once
#include <string>

class Bill {
 private:
  std::string name, serial;
  double sum;
  bool isPaid;

 public:
  Bill(std::string name, std::string serial, double sum, bool isPaid)
      : name(name), serial(serial), sum(sum), isPaid(isPaid) {}
  ~Bill(){};

  std::string getName() const { return name; }
  std::string getSerial() const { return serial; }
  double getSum() const { return sum; }
  bool getIsPaid() const { return isPaid; }

  std::string toString() const {
    return name + " " + serial + " " + std::to_string(sum) + " " +
           (isPaid ? "Paid" : "Not paid");
  }
};
