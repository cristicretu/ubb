#include "client.h"

std::string Client::toString() {
  return "Name: " + name + ", Income: " + std::to_string(income);
}

bool Person::isInterested(Dwelling dwelling) {
  return dwelling.getPrice() / 1000.00 <= this->totalIncome();
}

std::string Person::toString() { return "Person: " + Client::toString(); }

std::string Company::toString() {
  return "Company: " + Client::toString() +
         ", Money from investments: " + std::to_string(moneyFromInvestments);
}

bool Company::isInterested(Dwelling dwelling) {
  return dwelling.getPrice() / 1000.00 <= this->totalIncome() &&
         dwelling.isDwellingProfitable();
}