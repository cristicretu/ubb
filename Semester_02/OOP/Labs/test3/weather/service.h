#pragma once
#include <algorithm>
#include <iostream>

#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service(){};
  ~Service(){};

  std::vector<Weather> getWeather() const {
    auto weather = repo.getWeather();

    std::sort(weather.begin(), weather.end(),
              [](const Weather &a, const Weather &b) {
                return a.getStart() < b.getStart();
              });

    return weather;
  }

  std::vector<Weather> filterWeather(int prob) const {
    auto weather = repo.getWeather();

    std::vector<Weather> ans;

    for (auto x : weather) {
      if (x.getPrecipitation() < prob) {
        ans.push_back(x);
      }
    }

    return ans;
  }

  std::pair<int, std::vector<Weather>> getAnswer(
      int hour, std::string description) const {
    auto weather = repo.getWeather();
    int ans = 0;
    std::vector<Weather> rasp;

    std::cout << hour << " " << description << "\n";
    for (auto x : weather) {
      if (x.getDescription() == description) {
        if (x.getEnd() <= hour) {
          continue;
        }

        int duration = x.getEnd() - x.getStart();

        if (x.getStart() < hour) {
          duration -= (hour - x.getStart());
        }

        ans += duration;

        rasp.push_back(x);
      }
    }

    return {ans, rasp};
  }
};
