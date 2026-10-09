#pragma once
#include <iostream>
#include <vector>

#include "domain.h"

class Service {
 private:
  std::vector<HospitalDepartment *> departments;

 public:
  Service() {
    HospitalDepartment *neonantalUnitSanovil =
        new NeonatalUnit("Sanovil", 10, 20, 10, 8.5);
    HospitalDepartment *neonantalUnitReginaMaria =
        new NeonatalUnit("Regina Maria", 25, 7, 10, 9.5);

    HospitalDepartment *surgeryUnitSanovil = new Surgery("Sanovil", 30, 60);
    HospitalDepartment *surgeryUnitReginaMaria =
        new Surgery("Regina Maria", 10, 10);

    departments.push_back(neonantalUnitSanovil);
    departments.push_back(neonantalUnitReginaMaria);
    departments.push_back(surgeryUnitSanovil);
    departments.push_back(surgeryUnitReginaMaria);

    for (auto department : departments) {
      std::cout << department->toString() << std::endl;
    }
  };
  ~Service() {
    for (auto department : departments) {
      delete department;
    }
  };
  void addDepartment(std::string type, std::string hospitalName,
                     int numberOfDoctors, int numberOfPatients,
                     int numberOfMothers, int numberOfNewborns,
                     double averageGrade);
  std::vector<HospitalDepartment *> getDepartments() { return departments; }
  std::vector<HospitalDepartment *> getEfficientDepartments();
  void writeToFile(std::string filename);
};
