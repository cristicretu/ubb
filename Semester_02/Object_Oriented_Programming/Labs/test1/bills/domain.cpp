#include "domain.h"

#include <string>

Bill::Bill(std::string serial_number, std::string company, int day, int month,
           int year, bool paid)
    : serial_number{serial_number},
      company{company},
      day{day},
      month{month},
      year{year},
      paid{paid} {}

Bill::~Bill() {}

std::string Bill::getCompany() const { return this->company; }
std::string Bill::getSerialNumber() const { return this->serial_number; }
bool Bill::getPaidStatus() const { return this->paid; }
int Bill::getDay() const { return this->day; }
int Bill::getMonth() const { return this->month; }
int Bill::getYear() const { return this->year; }

void Bill::setSerialNumber(std::string s) { this->serial_number = s; }
void Bill::setCompany(std::string s) { this->company = s; }
void Bill::setPaidStatus(bool p) { this->paid = p; }
void Bill::setDay(int d) { this->day = d; }
void Bill::setMonth(int m) { this->month = m; }
void Bill::setYear(int y) { this->year = y; }