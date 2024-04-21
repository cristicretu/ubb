#include "domain.h"

bool Surgery::isEfficient() { return numberOfPatients / numberOfDoctors >= 2; }
std::string Surgery::toString() {
  return "Surgery at " + hospitalName + " has " +
         std::to_string(numberOfDoctors) + " doctors and " +
         std::to_string(numberOfPatients) + " patients.";
}

bool NeonatalUnit::isEfficient() {
  return averageGrade >= 8.5 && numberOfNewborns >= numberOfMothers;
}
std::string NeonatalUnit::toString() {
  return "Neonatal unit at " + hospitalName + " has " +
         std::to_string(numberOfDoctors) + " doctors, " +
         std::to_string(numberOfMothers) + " mothers, " +
         std::to_string(numberOfNewborns) +
         " newborns and an average grade of " + std::to_string(averageGrade) +
         ".";
}
