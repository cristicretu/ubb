#pragma once
#include <string>

class HospitalDepartment {
 protected:
  std::string hospitalName;
  int numberOfDoctors;

 public:
  HospitalDepartment(std::string hospitalName, int numberOfDoctors)
      : hospitalName{hospitalName}, numberOfDoctors{numberOfDoctors} {};
  virtual ~HospitalDepartment(){};
  virtual bool isEfficient() { return false; }
  virtual std::string toString() = 0;
  std::string getHospitalName() { return hospitalName; }
};

class Surgery : public HospitalDepartment {
 private:
  int numberOfPatients;

 public:
  Surgery(std::string hospitalName, int numberOfDoctors, int numberOfPacients)
      : HospitalDepartment{hospitalName, numberOfDoctors},
        numberOfPatients{numberOfPacients} {};
  ~Surgery(){};
  bool isEfficient() override;
  std::string toString() override;
};

class NeonatalUnit : public HospitalDepartment {
 private:
  int numberOfMothers;
  int numberOfNewborns;
  double averageGrade;

 public:
  NeonatalUnit(std::string hospitalName, int numberOfDoctors,
               int numberOfMothers, int numberOfNewborns, double averageGrade)
      : HospitalDepartment(hospitalName, numberOfDoctors),
        numberOfMothers(numberOfMothers),
        numberOfNewborns(numberOfNewborns),
        averageGrade(averageGrade) {}
  ~NeonatalUnit(){};
  bool isEfficient() override;
  std::string toString() override;
};