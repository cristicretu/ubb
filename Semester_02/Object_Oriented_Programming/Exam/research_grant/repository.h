#pragma once
#include <algorithm>
#include <iostream>
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
  };

  vector<Researcher> &getResearchers() { return researchers; }
  vector<Idea> &getIdeas() {
    sort(ideas.begin(), ideas.end(), [](const Idea &a, const Idea &b) {
      return a.getDuration() < b.getDuration();
    });

    return ideas;
  }

  void acceptIdea(Idea &idea) { idea.setStatus(1); }

  void addIdea(Idea idea) { ideas.emplace_back(idea); }

  Idea &findIdea(string title, string description) {
    for (auto &idea : ideas) {
      if (idea.getTitle() == title) {
        return idea;
      }
    }

    throw runtime_error("idea not found");
  }

  void changeDescription(Idea &idea, string newDesc) {
    idea.setDescription(newDesc);
  }

  void loadResearchers() {
    ifstream fin("../researchers.txt");
    string name, position;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, name, '|');
      getline(iss, position);

      researchers.emplace_back(Researcher(name, position));
    }
    fin.close();
  }
  void loadIdeas() {
    ifstream fin("../ideas.txt");
    string title, desc, status, name, year;

    string line;
    while (getline(fin, line)) {
      istringstream iss(line);

      getline(iss, title, '|');
      getline(iss, desc, '|');
      getline(iss, status, '|');
      getline(iss, name, '|');
      getline(iss, year);

      ideas.emplace_back(Idea(title, desc, stoi(status), name, stoi(year)));
    }

    fin.close();
  }

  void saveAcceptedIdeas() {
    ofstream fout("../accepted_ideas.txt");

    for (auto x : ideas) {
      if (x.getStatus() == 1) {
        fout << x.getTitle() << "  [" << x.getCreator() << "] "
             << x.getDuration() << "  " << x.getDescription() << '\n';
      }
    }
  }

  void saveIdea(Idea x) {
    ofstream fout("../" + x.getTitle() + ".txt");

    fout << x.getTitle() << "  [" << x.getCreator() << "] " << x.getDuration()
         << "  " << x.getDescription() << '\n';
  }
};
