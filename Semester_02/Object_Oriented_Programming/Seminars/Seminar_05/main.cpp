#include "CSVPlaylist.h"
#include "RepositoryExceptions.h"
#include "UI.h"

using namespace std;

int main() {
  try {
    Repository repo("../Songs.txt");
    FilePlaylist* p = new CSVPlaylist{};
    Service serv(repo, p, SongValidator{});
    UI ui(serv);
    ui.run();
    delete p;
  } catch (FileException&) {
    cout << "Repository file could not be opened! The application will "
            "terminate."
         << endl;
    return 1;
  }

  return 0;
}