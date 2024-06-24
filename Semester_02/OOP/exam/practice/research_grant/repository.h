#pragma once
#include <algorithm>
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

  vector<Researcher>& getResearchers() { return researchers; }
  vector<Idea>& getIdeas() {
    sort(ideas.begin(), ideas.end(), [](const Idea& a, const Idea& b) {
      return a.getDuration() < b.getDuration();
    });

    return ideas;
  }

  void addIdea(const Idea& idea) { ideas.push_back(idea); }
  int getIdeaIndex(const Idea& idea) {
    for (int i = 0; i < ideas.size(); i++) {
      if (ideas[i].getTitle() == idea.getTitle()) {
        return i;
      }
    }
    return -1;
  }

  void loadResearchers() {
    string name, position, line;

    ifstream fin(
        "../"
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
        "../"
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
        "../"
        "researchers.txt");

    for (const auto& researcher : researchers) {
      fout << researcher.getName() << "|" << researcher.getPosition() << endl;
    }
  }

  void saveIdeas() {
    ofstream fout(
        "../"
        "ideas.txt");

    for (const auto& idea : ideas) {
      fout << idea.getTitle() << "|" << idea.getDescription() << "|"
           << idea.getCreator() << "|" << idea.getStatus() << "|"
           << idea.getDuration() << endl;
    }
  }
};
