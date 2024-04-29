#pragma once
#include "Repository.h"
#include "Song.h"

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
  Song addedSong;
  Repository& repo;

 public:
  ActionAdd(Repository& repo, const Song& addedSong)
      : repo{repo}, addedSong{addedSong} {
    // this->repo.addSong(addedSong);
  }
  ~ActionAdd(){};
  void executeUndo() { this->repo.removeSong(this->addedSong); }
  void executeRedo() { this->repo.addSong(this->addedSong); }
};

class ActionRemove : public Action {
 private:
  Song removedSong;
  Repository& repo;

 public:
  ActionRemove(Repository& repo, const Song& removedSong)
      : repo{repo}, removedSong{removedSong} {
    // this->repo.removeSong(removedSong);
  }
  ~ActionRemove(){};
  void executeUndo() { this->repo.addSong(this->removedSong); }
  void executeRedo() { this->repo.removeSong(this->removedSong); }
};

class ActionUpdate : public Action {
 private:
  Song oldSong;
  Song newSong;
  Repository& repo;

 public:
  ActionUpdate(Repository& repo, const Song& oldSong, const Song& newSong)
      : repo{repo}, oldSong{oldSong}, newSong{newSong} {
    // this->repo.removeSong(oldSong);
    // this->repo.addSong(newSong);
  }
  ~ActionUpdate(){};
  void executeUndo() {
    this->repo.removeSong(this->newSong);
    this->repo.addSong(this->oldSong);
  }
  void executeRedo() {
    this->repo.removeSong(this->oldSong);
    this->repo.addSong(this->newSong);
  }
};
