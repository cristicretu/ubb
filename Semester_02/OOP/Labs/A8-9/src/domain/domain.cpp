#include "../../headers/domain/domain.h"

#include <iostream>

Dog::Dog(const std::string &breed, const std::string &name, int age,
         const std::string &photograph)
    /*
    The default constructor for the Dog class
    :param breed: The breed of the dog
    :param name: The name of the dog
    :param age: The age of the dog
    :param photograph: The photograph of the dog
    */
    : breed(breed), name(name), age(age), photograph(photograph) {}

Dog::~Dog() {}

Dog::Dog(const Dog &dog) {
  this->breed = dog.breed;
  this->name = dog.name;
  this->age = dog.age;
  this->photograph = dog.photograph;
}

std::string Dog::getBreed() const { return this->breed; }

std::string Dog::getName() const { return this->name; }

std::string Dog::getPhotograph() const { return this->photograph; }

int Dog::getAge() const { return this->age; }

bool Dog::operator==(const Dog &other) const {
  return breed == other.breed && name == other.name && age == other.age &&
         photograph == other.photograph;
}

void Dog::setBreed(const std::string &breed) { this->breed = breed; }

void Dog::setName(const std::string &name) { this->name = name; }

void Dog::setPhotograph(const std::string &photograph) {
  this->photograph = photograph;
}

void Dog::setAge(int age) { this->age = age; }

void Dog::print() const {
  std::cout << "Breed: " << this->breed << "\n";
  std::cout << "Name: " << this->name << "\n";
  std::cout << "Age: " << this->age << "\n";
  std::cout << "Photograph: " << this->photograph << "\n";
}

std::istream &operator>>(std::istream &input, Dog &dog) {
  std::string line;
  std::getline(input, line);

  std::vector<std::string> tokens = dog.split(line, " | ");

  try {
    DogValidator::validateDogIdentifiers(tokens[0], tokens[1],
                                         std::stoi(tokens[2]), tokens[3]);
  } catch (DogException &e) {
    for (auto &err : e.returnErrors()) {
      std::cout << err.what() << "\n";
    }
  }

  dog.setBreed(tokens[0]);
  dog.setName(tokens[1]);
  dog.setAge(std::stoi(tokens[2]));
  dog.setPhotograph(tokens[3]);

  return input;
}

std::ostream &operator<<(std::ostream &output, const Dog &dog) {
  output << dog.getBreed() << " | " << dog.getName() << " | " << dog.getAge()
         << " | " << dog.getPhotograph() << "\n";

  return output;
}