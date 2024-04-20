#include "service.h"

#include <fstream>

void Service::populateLists() {
  dwellings.push_back(Dwelling(100000, true));
  dwellings.push_back(Dwelling(200000, false));
  dwellings.push_back(Dwelling(300000, true));
  dwellings.push_back(Dwelling(400000, false));
  dwellings.push_back(Dwelling(500000, true));

  clients.push_back(new Person("John", 10000));
  clients.push_back(new Person("Alice", 20000));
  clients.push_back(new Person("Bob", 30000));

  clients.push_back(new Company("Company1", 100000, 10000));
  clients.push_back(new Company("Company2", 200000, 20000));
  clients.push_back(new Company("Company3", 300000, 30000));
}

Dwelling Service::addDwelling(double price, bool isProfitable) {
  Dwelling dwelling(price, isProfitable);
  dwellings.push_back(dwelling);
  return dwelling;
}

void Service::addClient(Client *client) { clients.push_back(client); }

void Service::removeClient(std::string name) {
  for (auto it = clients.begin(); it != clients.end(); ++it) {
    if ((*it)->getName() == name) {
      clients.erase(it);
      break;
    }
  }
}

std::vector<Client *> Service::interestedClients(Dwelling dwelling) {
  std::vector<Client *> interested;
  for (auto client : clients) {
    if (client->isInterested(dwelling)) {
      interested.push_back(client);
    }
  }
  return interested;
}

void Service::writeToFile(std::string filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    for (auto client : clients) {
      file << client->toString() << std::endl;
    }
    file.close();
  }
}