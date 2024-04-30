#pragma once
#include <iostream>
#include <sstream>
#include <string>

#include "DogValidator.h"

class Dog {
 private:
  std::string breed, name, photograph;
  int age;

 public:
  Dog(const std::string &breed = "", const std::string &name = "", int age = 0,
      const std::string &photograph = "");
  Dog(const Dog &dog);
  ~Dog();

  std::string getBreed() const;
  std::string getName() const;
  std::string getPhotograph() const;
  int getAge() const;

  bool operator==(const Dog &other) const;
  friend std::istream &operator>>(std::istream &input, Dog &dog);
  friend std::ostream &operator<<(std::ostream &output, const Dog &dog);

  void setBreed(const std::string &breed);
  void setName(const std::string &name);
  void setPhotograph(const std::string &photograph);
  void setAge(int age);

  void print() const;

  std::vector<std::string> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
      token = s.substr(pos_start, pos_end - pos_start);
      pos_start = pos_end + delim_len;
      res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
  }
};