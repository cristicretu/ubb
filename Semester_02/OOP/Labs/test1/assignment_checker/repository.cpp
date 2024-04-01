#include <repository.h>

#include <fstream>
#include <sstream>

Repository::Repository() { this->data = std::vector<Assignment>(); }

Repository::~Repository() {}

void Repository::addAssignment(Assignment a) { this->data.push_back(a); }

std::vector<Assignment> Repository::getAssignments() const {
  return this->data;
}

void Repository::readFromFile(std::string filename) {
  std::ifstream file(filename);
  std::string line;

  if (!file.is_open()) {
    throw std::runtime_error("Could not open file");
  }

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string token;
    std::string data[3];
    int index = 0;

    while (std::getline(iss, token, ',')) {
      if (index < 3) {
        data[index++] = token;
      }
    }

    if (index == 3) {
      int id = std::stoi(data[0]);
      std::string name = data[1];
      std::string solution = data[2];

    } else {
    }
  }
}