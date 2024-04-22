#include "domain.h"

Sensor::~Sensor() {}

bool TemperatureSensor::sendAlert() {
  int ans = 0;
  for (auto a : this->recordings) {
    if (a > 35 || a < 10) {
      ans += 1;
      if (ans == 2) {
        break;
      }
    }
  }
  return ans == 2;
}

std::string TemperatureSensor::toString() {
  std::string res = "Temperature sensor\n";
  res += "Producer: " + this->producer + "\n";
  res += "Recordings: ";
  for (auto a : this->recordings) {
    res += std::to_string(a) + " ";
  }
  res += "\n";
  res += "Diameter: " + std::to_string(this->diameter) + "\n";
  res += "Length: " + std::to_string(this->length) + "\n";
  return res;
}

bool HumiditySensor::sendAlert() {
  int ans = 0;
  for (auto a : this->recordings) {
    if (a > 85 || a < 30) {
      ans += 1;
      if (ans == 2) {
        break;
      }
    }
  }
  return ans == 2;
}

std::string HumiditySensor::toString() {
  std::string res = "Humidity sensor\n";
  res += "Producer: " + this->producer + "\n";
  res += "Recordings: ";
  for (auto a : this->recordings) {
    res += std::to_string(a) + " ";
  }
  res += "\n";
  return res;
}

bool SmokeSensor::sendAlert() {
  for (auto a : this->recordings) {
    if (a > 1600) {
      return true;
    }
  }
  return false;
}

std::string SmokeSensor::toString() {
  std::string res = "Smoke sensor\n";
  res += "Producer: " + this->producer + "\n";
  res += "Recordings: ";
  for (auto a : this->recordings) {
    res += std::to_string(a) + " ";
  }
  res += "\n";
  return res;
}
