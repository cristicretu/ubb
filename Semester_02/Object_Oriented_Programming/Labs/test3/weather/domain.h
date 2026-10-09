#pragma once
#include <string>

class Weather {
 private:
  int start, end, temp, precipitation;
  std::string description;

 public:
  Weather(int start, int end, int temp, int precipitation,
          std::string description)
      : start(start),
        end(end),
        temp(temp),
        precipitation(precipitation),
        description(description){};
  ~Weather(){};

  int getStart() const { return start; }
  int getEnd() const { return end; }
  int getTemp() const { return temp; }
  int getPrecipitation() const { return precipitation; }
  std::string getDescription() const { return description; }

  std::string toString() const {
    return std::to_string(start) + " <-> " + std::to_string(end) + "  |  " +
           std::to_string(temp) + "ˇ C  | " + std::to_string(precipitation) +
           "% |  " + description;
  }
};
