#pragma once
#include <iostream>

#include "repository.h"
#include "subject.h"

class Service : public Subject {
 private:
  Repository &repo;

 public:
  Service(Repository &repo) : repo{repo} {}
  ~Service(){};

  void addVolunteer(std::string name, std::string email,
                    std::string departmentName, std::string interests) {
    Volunteer volunteer(name, email, departmentName, interests);
    repo.addVolunteer(volunteer);
  }

  void assignVolunteer(int index, std::string department) {
    repo.assignVolunteer(index, department);
  }

  std::vector<Department> &getDepartments() { return repo.getDepartments(); }

  std::vector<Volunteer> &getVolunteers() { return repo.getVolunteers(); }

  std::vector<std::string> getVolunteersFromDepartment(
      std::string departmentName) {
    std::vector<std::string> volunteers;
    for (Volunteer volunteer : repo.getVolunteers()) {
      if (volunteer.getDepartmentName() == departmentName) {
        volunteers.push_back(volunteer.getName());
      }
    }
    return volunteers;
  }

  std::vector<std::string> getUnassignedVolunteers() {
    std::vector<std::string> volunteers;
    for (Volunteer volunteer : repo.getVolunteers()) {
      if (volunteer.getDepartmentName() == "") {
        volunteers.push_back(volunteer.getName());
      }
    }
    return volunteers;
  }

  std::vector<std::string> getMostSuitable(std::string description) {
    std::vector<std::string> volunteers;
    for (Volunteer volunteer : repo.getVolunteers()) {
      if (volunteer.getInterests() == description) {
        volunteers.push_back(volunteer.getName());
      }
    }
    return volunteers;
  }
};
