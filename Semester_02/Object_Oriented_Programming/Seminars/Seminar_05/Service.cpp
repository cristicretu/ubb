#include "Service.h"

#include <algorithm>

#include "FilePlaylist.h"
#include "RepositoryExceptions.h"

using namespace std;

void Service::updateSongFromRepository(
    const std::string& artist, const std::string& title, double minutes,
    double seconds, const std::string& newArtist, const std::string& newTitle,
    double newMinutes, double newSeconds, const std::string& newSource) {
  Song oldSong = this->repo.findByArtistAndTitle(artist, title);
  Song newSong{newArtist, newTitle, Duration{newMinutes, newSeconds},
               newSource};
  this->repo.updateSong(oldSong, newSong);

  unique_ptr<Action> action =
      make_unique<ActionUpdate>(this->repo, oldSong, newSong);

  this->undoActions.push(move(action));
  while (!this->redoActions.empty()) this->redoActions.pop();
}

void Service::addSongToRepository(const std::string& artist,
                                  const std::string& title, double minutes,
                                  double seconds, const std::string& source) {
  Song s{artist, title, Duration{minutes, seconds}, source};
  this->validator.validate(s);
  this->repo.addSong(s);

  unique_ptr<Action> action = make_unique<ActionAdd>(this->repo, s);

  this->undoActions.push(move(action));
  while (!this->redoActions.empty()) this->redoActions.pop();
}

void Service::removeSongFromRepository(const std::string& artist,
                                       const std::string& title) {
  Song s = this->repo.findByArtistAndTitle(artist, title);
  this->repo.removeSong(s);

  unique_ptr<Action> action = make_unique<ActionRemove>(this->repo, s);

  this->undoActions.push(move(action));
  while (!this->redoActions.empty()) this->redoActions.pop();
}

void Service::addSongToPlaylist(const Song& song) {
  if (this->playList == nullptr) return;
  this->playList->add(song);

  unique_ptr<Action> action = make_unique<ActionAdd>(this->repo, song);

  this->undoActions.push(move(action));
  while (!this->redoActions.empty()) this->redoActions.pop();
}

void Service::addAllSongsByArtistToPlaylist(const std::string& artist) {
  vector<Song> songs = this->repo.getSongs();
  int nSongs = static_cast<int>(
      count_if(songs.begin(), songs.end(),
               [artist](const Song& s) { return s.getArtist() == artist; }));

  vector<Song> songsByArtist(nSongs);
  copy_if(songs.begin(), songs.end(), songsByArtist.begin(),
          [artist](const Song& s) { return s.getArtist() == artist; });

  for (auto s : songsByArtist) this->playList->add(s);
}

void Service::startPlaylist() {
  if (this->playList == nullptr) return;
  this->playList->play();
}

void Service::nextSongPlaylist() {
  if (this->playList == nullptr) return;
  this->playList->next();
}

void Service::savePlaylist(const std::string& filename) {
  if (this->playList == nullptr) return;

  this->playList->setFilename(filename);
  this->playList->writeToFile();
}

void Service::openPlaylist() const {
  if (this->playList == nullptr) return;

  this->playList->displayPlaylist();
}
