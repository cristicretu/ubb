#pragma once
#include <string>

class Equation {
 private:
  double a, b, c;

 public:
  Equation(double a, double b, double c) : a(a), b(b), c(c){};
  ~Equation(){};

  double getA() const { return a; }
  double getB() const { return b; }
  double getC() const { return c; }

  double getDiscriminant() const { return b * b - 4 * a * c; }
  std::string toString() const {
    return (a != 0 ? std::to_string(a) + "x^2" : "") + (b > 0 ? "+" : "") +
           (b != 0 ? std::to_string(b) + "x" : "") + (c > 0 ? "+" : "") +
           (c != 0 ? std::to_string(c) : "");
  }

  void setA(double a) { this->a = a; }
  void setB(double b) { this->b = b; }
  void setC(double c) { this->c = c; }

  std::string solve() const {
    double d = getDiscriminant();
    if (d < 0) {
      return "x1 = " + std::to_string(-b / (2 * a)) + " + " +
             std::to_string(sqrt(-d) / (2 * a)) +
             "i, x2 = " + std::to_string(-b / (2 * a)) + " - " +
             std::to_string(sqrt(-d) / (2 * a)) + "i";
    } else if (d == 0) {
      return "x = " + std::to_string(-b / (2 * a));
    } else {
      return "x1 = " + std::to_string((-b + sqrt(d)) / (2 * a)) +
             ", x2 = " + std::to_string((-b - sqrt(d)) / (2 * a));
    }
  }

  bool hasRealRoots() const { return getDiscriminant() >= 0; }
};
