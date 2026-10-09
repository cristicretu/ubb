#pragma once

#include "repository.h"
#include "subject.h"

class Session : public Subject {
 private:
  Repository &repo;

 public:
  Session(Repository &repo) : repo(repo){};

  void addIdea(string title, string description, string creator, bool status,
               int duration) {
    if (title.empty() || description.empty() || creator.empty()) {
      throw invalid_argument("All fields must be filled in!");
    }

    if (duration < 1 || duration > 3) {
      throw invalid_argument("Duration must be between 1 and 3!");
    }

    if (status != 0) {
      throw invalid_argument("Status must be 0");
    }

    if (repo.getIdeaIndex(
            Idea(title, description, creator, status, duration)) != -1) {
      throw invalid_argument("Idea already exists!");
    }

    auto idea = Idea(title, description, creator, status, duration);
    repo.addIdea(idea);
    notify();
  }

  void acceptIdea(string title) {
    Idea idea = Idea(title, "", "", 0, 0);
    int index = repo.getIdeaIndex(idea);

    if (index == -1) {
      throw invalid_argument("Idea does not exist!");
    }

    repo.getIdeas()[index].setStatus(1);
    notify();
  }

  vector<Idea> &getIdeas() { return repo.getIdeas(); }
  vector<Researcher> &getResearchers() { return repo.getResearchers(); }
};