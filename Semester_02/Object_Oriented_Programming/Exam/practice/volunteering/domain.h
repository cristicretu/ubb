#pragma once
#include <string>
#include <vector>

class Department {
 private:
  std::string name, description;

 public:
  Department(std::string name, std::string description)
      : name(name), description(description){};

  std::string getName() { return name; }
  std::string getDescription() { return description; }
};

class Volunteer {
 private:
  std::string name, email, interests, departmentName;

 public:
  Volunteer(std::string name, std::string email, std::string departmentName,
            std::string interests)
      : name(name),
        email(email),
        departmentName(departmentName),
        interests(interests){};

  std::string getName() { return name; }
  std::string getEmail() { return email; }
  std::string getDepartmentName() { return departmentName; }
  std::string getInterests() { return interests; }
  std::string getVolunteer() {
    return name + " " + email + " " + departmentName + " " + interests;
  }
  void setDepartmentName(std::string departmentName) {
    this->departmentName = departmentName;
  }
};