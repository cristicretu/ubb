#pragma once
#include <fstream>
#include <iostream>
#include <sstream>

#include "domain.h"

class Repository {
 private:
  std::vector<Department> departments;
  std::vector<Volunteer> volunteers;

 public:
  Repository() {
    loadDepartments();
    loadVolunteers();
  };
  ~Repository(){};
  void loadDepartments() {
    std::ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/volunteering/"
        "departments.txt");
    std::string name, description;
    std::string line;

    if (!fin.is_open()) {
      std::cout << "File not found" << std::endl;
      return;
    }

    std::cout << "Loading departments" << std::endl;
    while (std::getline(fin, line)) {
      std::cout << line << std::endl;
      std::istringstream iss(line);
      std::getline(iss, name, '|');
      std::getline(iss, description);
      departments.push_back(Department(name, description));
    }
  }

  void loadVolunteers() {
    std::ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/volunteering/"
        "volunteers.txt");
    std::string name, email, departmentName, interests;
    std::string line;
    while (std::getline(fin, line)) {
      std::istringstream iss(line);
      std::getline(iss, name, '|');
      std::getline(iss, email, '|');
      std::getline(iss, interests, '|');
      std::getline(iss, departmentName);

      volunteers.push_back(Volunteer(name, email, departmentName, interests));
    }
  }

  Department getDepartmentByName(std::string name) {
    for (Department department : departments) {
      if (department.getName() == name) {
        return department;
      }
    }
    return Department("", "");
  }

  void addVolunteer(Volunteer volunteer) { volunteers.push_back(volunteer); }
  void assignVolunteer(int index, std::string department) {
    volunteers[index].setDepartmentName(department);
  }

  std::vector<Department> &getDepartments() { return departments; }
  std::vector<Volunteer> &getVolunteers() { return volunteers; }
};
