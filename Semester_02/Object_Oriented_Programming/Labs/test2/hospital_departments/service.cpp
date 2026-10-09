#include "service.h"

#include <algorithm>
#include <fstream>

void Service::addDepartment(std::string type, std::string hospitalName,
                            int numberOfDoctors, int numberOfPatients,
                            int numberOfMothers, int numberOfNewborns,
                            double averageGrade) {
  if (type == "Surgery") {
    departments.push_back(
        new Surgery(hospitalName, numberOfDoctors, numberOfPatients));
  } else if (type == "Neonatal Unit") {
    departments.push_back(new NeonatalUnit(hospitalName, numberOfDoctors,
                                           numberOfMothers, numberOfNewborns,
                                           averageGrade));
  }
}

std::vector<HospitalDepartment*> Service::getEfficientDepartments() {
  std::vector<HospitalDepartment*> efficientDepartments;
  for (auto department : departments) {
    if (department != nullptr && department->isEfficient()) {
      efficientDepartments.push_back(department);
    }
  }
  return efficientDepartments;
}

void Service::writeToFile(std::string filename) {
  std::ofstream file(filename);
  std::sort(departments.begin(), departments.end(),
            [](HospitalDepartment* a, HospitalDepartment* b) {
              return a->getHospitalName() < b->getHospitalName();
            });
  for (auto department : departments) {
    file << department->toString() << "\n";
  }
  file.close();
}