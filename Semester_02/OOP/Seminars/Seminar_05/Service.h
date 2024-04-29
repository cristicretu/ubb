#pragma once
#include <memory>
#include <stack>

#include "Action.h"
#include "FilePlaylist.h"
#include "Repository.h"
#include "SongValidator.h"

class Service {
 private:
  Repository repo;
  FilePlaylist* playList;
  SongValidator validator;
  std::stack<std::unique_ptr<Action>> undoActions;
  std::stack<std::unique_ptr<Action>> redoActions;

 public:
  Service(const Repository& r, FilePlaylist* p, SongValidator v)
      : repo{r}, playList{p}, validator{v} {
    this->undoActions = std::stack<std::unique_ptr<Action>>();
    this->redoActions = std::stack<std::unique_ptr<Action>>();
  }

  Repository getRepo() const { return repo; }
  PlayList* getPlaylist() const { return playList; }

  /*
          Adds a song with the given data to the song repository.
          Throws: SongException - if the song is not valid
                          DuplicateSongException - if there is another song with
     the same artist and title Throws: FileException - if the repository file
     cannot be opened.
  */
  void addSongToRepository(const std::string& artist, const std::string& title,
                           double minutes, double seconds,
                           const std::string& source);

  void removeSongFromRepository(const std::string& artist,
                                const std::string& title);

  /*
          Adds a given song to the current playlist.
          Input: song - Song, the song must belong to the repository.
          Output: the song is added to the playlist.
  */
  void addSongToPlaylist(const Song& song);

  // Adds all the songs from the repository, that have the given artist, to the
  // current playlist.
  void addAllSongsByArtistToPlaylist(const std::string& artist);

  void startPlaylist();
  void nextSongPlaylist();

  /*
          Saves the playlist.
          Throws: FileException - if the given file cannot be opened.
  */
  void savePlaylist(const std::string& filename);

  /*
  Opens the playlist, with an appropriate application.
  Throws: FileException - if the given file cannot be opened.
  */
  void openPlaylist() const;

  void undo() {
    if (this->undoActions.empty()) {
      throw std::runtime_error("No more undo actions!");
    }
    std::unique_ptr<Action> action = std::move(this->undoActions.top());
    action->executeUndo();
    this->redoActions.push(std::move(action));
    this->undoActions.pop();
  }
  void redo() {
    if (this->redoActions.empty()) {
      throw std::runtime_error("No more redo actions!");
    }
    std::unique_ptr<Action> action = std::move(this->redoActions.top());
    action->executeRedo();
    this->undoActions.push(std::move(action));
    this->redoActions.pop();
  }
};
