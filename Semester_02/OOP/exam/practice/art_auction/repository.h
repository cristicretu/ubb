#pragma once
#include <fstream>
#include <sstream>

#include "domain.h"

class Repository {
 private:
  vector<User> users;
  vector<Item> items;

 public:
  Repository() {
    loadUsers();
    loadItems();
  }
  ~Repository(){};

  vector<User> getUsers() const { return users; }
  vector<Item> getItems() const { return items; }

  void loadUsers() {
    ifstream file("../users.txt");
    string line;
    while (getline(file, line)) {
      istringstream iss(line);

      string name, type;
      string idstr;

      getline(iss, name, '|');
      getline(iss, idstr, '|');
      getline(iss, type);
      users.emplace_back(User(name, stoi(idstr), type));
    }
    file.close();
  }

  void loadItems() {
    ifstream file("../items.txt");
    string line;

    vector<tuple<int, string, int>> oferte;
    while (getline(file, line)) {
      istringstream iss(line);

      string name, category;
      string priceStr;

      getline(iss, name, '|');
      getline(iss, category, '|');
      getline(iss, priceStr);

      string offers;
      getline(iss, offers);

      istringstream issOffers(offers);

      while (getline(issOffers, offers, '|')) {
        istringstream messi(offers);

        string idStr, date, priceStr;
        getline(messi, idStr, ',');
        getline(messi, date, ',');
        getline(messi, priceStr);

        oferte.emplace_back(make_tuple(stoi(idStr), date, stoi(priceStr)));
      }
      items.emplace_back(Item(name, category, stoi(priceStr), oferte));
      oferte.clear();
    }
    file.close();
  }
};
