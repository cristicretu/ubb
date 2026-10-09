#pragma once
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "domain.h"

class DogException : public std::exception {
 private:
  std::vector<std::invalid_argument> err;

 public:
  DogException(std::vector<std::invalid_argument> err) { this->err = err; }
  DogException(std::string err) {
    this->err.push_back(std::invalid_argument(err));
  }
  std::vector<std::invalid_argument> returnErrors() { return this->err; }
};

class DogValidator {
 public:
  static void validateDogName(const std::string &);
  static void validateDogBreed(const std::string &);
  static void validateDogYear(const int &);
  static void validateDogPhoto(const std::string &);
  static void validateDogIdentifiers(const std::string &, const std::string &,
                                     const int &, const std::string &);
};