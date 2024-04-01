#pragma once
#include <string>

class Assignment {
 private:
  std::string name, solution;
  int id;

 public:
  Assignment(std::string name, std::string solutuion, int id);
  ~Assignment();
  std::string getName() const;
  std::string getSolution() const;
  int getId() const;
  void setName(std::string name);
  void setSolution(std::string solution);
  void setId(int id);
  bool operator==(const Assignment& a);
};