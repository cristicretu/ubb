#include "service.h"

#include <algorithm>
#include <sstream>

Service::Service() {
  this->repo = Repository();
  this->repo.readFromFile("assignments.txt");
}

Service::~Service() {}

void Service::addAssignment(std::string name, std::string solution, int id) {
  Assignment a = Assignment(name, solution, id);

  if (name == "" || solution == "" || id < 0) {
    throw std::invalid_argument("Invalid assignment data");
  }

  this->repo.addAssignment(a);
}

std::vector<Assignment> Service::getAssignments() const {
  std::vector<Assignment> assignments = this->repo.getAssignments();

  sort(assignments.begin(), assignments.end(),
       [](Assignment a, Assignment b) { return a.getId() > b.getId(); });

  return assignments;
}

std::vector<std::vector<Assignment>> Service::dishnestyCheck() const {
  std::vector<Assignment> assignments = this->getAssignments();
  std::vector<std::vector<Assignment>> result;

  // at least 20% of the words from the solution are common
  for (int i = 0; i < assignments.size(); i++) {
    std::vector<Assignment> group;
    group.push_back(assignments[i]);

    for (int j = i + 1; j < assignments.size(); j++) {
      int common = 0;
      std::string solution1 = assignments[i].getSolution();
      std::string solution2 = assignments[j].getSolution();

      std::istringstream iss1(solution1);
      std::istringstream iss2(solution2);

      std::string word1;
      std::string word2;

      while (iss1 >> word1) {
        while (iss2 >> word2) {
          if (word1 == word2) {
            common++;
          }
        }
      }

      if (common >= 0.2 * solution1.size()) {
        group.push_back(assignments[j]);
      }
    }

    if (group.size() > 1) {
      result.push_back(group);
    }
  }
  return result;
}