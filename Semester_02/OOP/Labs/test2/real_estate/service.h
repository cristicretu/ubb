#pragma once
#include <vector>

#include "client.h"

class Service {
 private:
  std::vector<Client *> clients;
  std::vector<Dwelling> dwellings;

 public:
  Service() { populateLists(); }
  ~Service(){};
  void populateLists();
  Dwelling addDwelling(double price, bool isProfitable);
  std::vector<Dwelling> getDwellings() const { return dwellings; }
  void addClient(Client *client);
  void removeClient(std::string name);
  std::vector<Client *> getClients() const { return clients; }
  std::vector<Client *> interestedClients(Dwelling dwelling);
  void writeToFile(std::string filename);
};
