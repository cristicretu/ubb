#include <string>
#pragma once

class Bill {
 private:
  std::string serial_number, company;
  int day, month, year;
  bool paid;

 public:
  Bill(std::string serial_number, std::string company, int day, int month,
       int year, bool paid);

  ~Bill();

  std::string getSerialNumber() const;
  std::string getCompany() const;
  bool getPaidStatus() const;
  int getDay() const;
  int getMonth() const;
  int getYear() const;

  void setSerialNumber(std::string s);
  void setCompany(std::string s);
  void setPaidStatus(bool p);
  void setDay(int d);
  void setMonth(int m);
  void setYear(int y);
};