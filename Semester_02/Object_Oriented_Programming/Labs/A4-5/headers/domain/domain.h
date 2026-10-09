#pragma once
#include <iostream>
#include <string>

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

  void setBreed(const std::string &breed);
  void setName(const std::string &name);
  void setPhotograph(const std::string &photograph);
  void setAge(int age);

  void print() const;
};