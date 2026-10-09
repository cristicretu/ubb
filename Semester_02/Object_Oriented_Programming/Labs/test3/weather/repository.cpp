#include "repository.h"

void Repository::loadData() {
  std::ifstream fin("../weather.txt");

  std::string line;
  std::string start, end, temp, prec, description;

  char sep = ';';

  while (std::getline(fin, line)) {
    std::istringstream iss(line);

    std::getline(iss, start, sep);
    std::getline(iss, end, sep);
    std::getline(iss, temp, sep);
    std::getline(iss, prec, sep);
    std::getline(iss, description, sep);

    data.push_back(Weather(std::stoi(start), std::stoi(end), std::stoi(temp),
                           std::stoi(prec), description));
  }
}

std::vector<Weather> Repository::getWeather() const { return data; }