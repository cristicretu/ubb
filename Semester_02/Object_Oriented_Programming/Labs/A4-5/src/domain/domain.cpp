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

// Destructor
Dog::~Dog() {}

// Copy constructor
Dog::Dog(const Dog &dog) {
  this->breed = dog.breed;
  this->name = dog.name;
  this->age = dog.age;
  this->photograph = dog.photograph;
}

// Getter
std::string Dog::getBreed() const { return this->breed; }

std::string Dog::getName() const { return this->name; }

std::string Dog::getPhotograph() const { return this->photograph; }

int Dog::getAge() const { return this->age; }

// Overloading the equality operator
// Two dogs are equal if they have the same breed, name, age and photograph
bool Dog::operator==(const Dog &other) const {
  return breed == other.breed && name == other.name && age == other.age &&
         photograph == other.photograph;
}

// Setters
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