#include "domain.h"

Assignment::Assignment(std::string name, std::string solution, int id)
    : name{name}, solution{solution}, id{id} {}

Assignment::~Assignment() {}

std::string Assignment::getName() const { return name; }

std::string Assignment::getSolution() const { return solution; }

int Assignment::getId() const { return id; }

void Assignment::setName(std::string name) { this->name = name; }

void Assignment::setSolution(std::string solution) {
  this->solution = solution;
}

void Assignment::setId(int id) { this->id = id; }

bool Assignment::operator==(const Assignment& a) {
  return this->name == a.name && this->solution == a.solution &&
         this->id == a.id;
}