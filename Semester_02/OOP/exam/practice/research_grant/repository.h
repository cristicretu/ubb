#pragma once
#include <fstream>
#include <sstream>
#include <vector>

#include "domain.h"

class Repository {
 private:
  vector<Researcher> researchers;
  vector<Idea> ideas;

 public:
  Repository() {
    loadResearchers();
    loadIdeas();
  }
  ~Repository() {
    saveResearchers();
    saveIdeas();
  }

  void loadResearchers() {
    string name, position, line;

    ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/research_grant/"
        "researchers.txt");

    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, position);

      researchers.emplace_back(Researcher(name, position));
    }
  }

  void loadIdeas() {
    string title, description, creator, status, duration, line;

    ifstream fin(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/research_grant/"
        "ideas.txt");

    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, title, '|');
      getline(iss, description, '|');
      getline(iss, creator, '|');
      getline(iss, status, '|');
      getline(iss, duration);

      int accepted = stoi(status);

      ideas.emplace_back(
          Idea(title, description, creator, accepted == 1, stoi(duration)));
    }
  }

  void saveResearchers() {
    ofstream fout(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/research_grant/"
        "researchers.txt");

    for (const auto& researcher : researchers) {
      fout << researcher.getName() << "|" << researcher.getPosition() << endl;
    }
  }

  void saveIdeas() {
    ofstream fout(
        "/Users/huge/fun/ubb/Semester_02/OOP/exam/practice/research_grant/"
        "ideas.txt");

    for (const auto& idea : ideas) {
      fout << idea.getTitle() << "|" << idea.getDescription() << "|"
           << idea.getCreator() << "|" << idea.getStatus() << "|"
           << idea.getDuration() << endl;
    }
  }
};
