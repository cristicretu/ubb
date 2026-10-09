#pragma once
#include "../repository/repository.h"
#include "domain.h"

class Action {
 protected:
 public:
  Action(){};
  virtual ~Action(){};

  virtual void executeUndo() = 0;
  virtual void executeRedo() = 0;
};

class ActionAdd : public Action {
 private:
  Dog addedDog;
  Repository& repo;

 public:
  ActionAdd(Repository& repo, const Dog& addedDog)
      : repo{repo}, addedDog{addedDog} {}
  ~ActionAdd(){};
  void executeUndo() {
    int index = this->repo.findDog(this->addedDog);
    this->repo.removeDog(index);
  }
  void executeRedo() { this->repo.addDog(this->addedDog); }
};

class ActionRemove : public Action {
 private:
  Dog removedDog;
  Repository& repo;

 public:
  ActionRemove(Repository& repo, const Dog& removedDog)
      : repo{repo}, removedDog{removedDog} {}
  ~ActionRemove(){};
  void executeUndo() { this->repo.addDog(this->removedDog); }
  void executeRedo() {
    int index = this->repo.findDog(this->removedDog);
    this->repo.removeDog(index);
  }
};

class ActionUpdate : public Action {
 private:
  Dog oldDog;
  Dog newDog;
  Repository& repo;

 public:
  ActionUpdate(Repository& repo, const Dog& oldDog, const Dog& newDog)
      : repo{repo}, oldDog{oldDog}, newDog{newDog} {}
  ~ActionUpdate(){};
  void executeUndo() {
    int index = this->repo.findDog(this->newDog);
    this->repo.removeDog(index);
    this->repo.addDog(this->oldDog);
  }
  void executeRedo() {
    int index = this->repo.findDog(this->oldDog);
    this->repo.removeDog(index);
    this->repo.addDog(this->newDog);
  }
};