#pragma once
#include "dwelling.h"

class Client {
 protected:
  std::string name;
  double income;

 public:
  Client(std::string name = "", double income = 0)
      : name(name), income(income){};
  virtual ~Client(){};
  virtual double totalIncome() { return income; }
  virtual bool isInterested(Dwelling dwelling) { return false; }
  virtual std::string toString();
  std::string getName() const { return name; }
};

class Person : public Client {
 public:
  Person(std::string name, double income) : Client(name, income){};
  bool isInterested(Dwelling dwelling) override;
  std::string toString() override;
};

class Company : public Client {
 private:
  double moneyFromInvestments;

 public:
  Company(std::string name, double income, double moneyFromInvestments)
      : Client(name, income), moneyFromInvestments(moneyFromInvestments){};
  double totalIncome() override { return income + moneyFromInvestments; }
  std::string toString() override;
  bool isInterested(Dwelling dwelling) override;
};