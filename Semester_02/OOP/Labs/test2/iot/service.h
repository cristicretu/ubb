#pragma once
#include <fstream>

#include "domain.h"

class Device {
 private:
  bool hasWifi;
  std::vector<Sensor*> sensors;

 public:
  Device() {
    this->hasWifi = false;
    sensors.push_back(new TemperatureSensor("producer1", {10, 20, 30}, 2, 40));
    sensors.push_back(new HumiditySensor("producer2", {40, 50, 60}));
    sensors.push_back(new SmokeSensor("producer3", {1000, 2000, 3000}));
  };
  ~Device() {
    for (auto s : sensors) {
      delete s;
    }
  };
  void setHasWifi(bool hasWifi) { this->hasWifi = hasWifi; }
  bool getHasWifi() { return this->hasWifi; }
  std::vector<Sensor*> getSensors() { return this->sensors; }
  std::vector<Sensor*> getAlerts();
  void addSensor(std::string type, std::string producer,
                 std::vector<double> recordings, double diameter,
                 double length);
  void writeToFile(std::string filename);
};
