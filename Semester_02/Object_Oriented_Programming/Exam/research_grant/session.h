#pragma once
#include "repository.h"

class Service {
 private:
  Repository &repo;

 public:
  Service(Repository &repo) : repo(repo){};

  vector<Researcher> getResearchers() { return repo.getResearchers(); }
  vector<Idea> &getIdeas() { return repo.getIdeas(); }

  void addIdea(string title, string desc, bool status, string creator,
               int duration) {
    if (duration != 1 && duration != 2 && duration && duration != 3 ||
        duration == 0 || title.empty()) {
      throw runtime_error("Invalid idea!!");
    }

    auto idea = Idea(title, desc, status, creator, duration);

    repo.addIdea(idea);
  }

  void saveAcceptedIdeas() { repo.saveAcceptedIdeas(); }

  void acceptIdea(string title, string description) {
    auto &idea = repo.findIdea(title, description);

    if (idea.getStatus() == 1) {
      throw runtime_error("Idea is already accepted!");
    }

    repo.acceptIdea(idea);
  }

  void changeDescription(string title, string description, string newDesc) {
    auto &idea = repo.findIdea(title, description);

    if (newDesc.empty()) {
      throw runtime_error("Invalid description");
    }

    repo.changeDescription(idea, newDesc);
  }

  void saveIdeaToFile(string title, string description) {
    auto &idea = repo.findIdea(title, description);

    repo.saveIdea(idea);
  }
};
