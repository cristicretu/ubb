#include "service.h"

std::vector<Sensor*> Device::getAlerts() {
  std::vector<Sensor*> ans;
  for (auto s : sensors) {
    if (s->sendAlert()) {
      ans.push_back(s);
    }
  }
  return ans;
}

void Device::addSensor(std::string type, std::string producer,
                       std::vector<double> recordings, double diameter,
                       double length) {
  if (type == "temperature") {
    sensors.push_back(
        new TemperatureSensor(producer, recordings, diameter, length));
  } else if (type == "humidity") {
    sensors.push_back(new HumiditySensor(producer, recordings));
  } else if (type == "smoke") {
    sensors.push_back(new SmokeSensor(producer, recordings));
  }
}

void Device::writeToFile(std::string filename) {
  std::ofstream f(filename);
  for (auto s : sensors) {
    f << s->toString();
  }
  f.close();
}