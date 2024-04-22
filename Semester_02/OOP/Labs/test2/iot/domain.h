#pragma once
#include <string>
#include <vector>

class Sensor {
 protected:
  std::string producer;
  std::vector<double> recordings;

 public:
  Sensor(std::string producer = "", std::vector<double> recordings = {})
      : producer(producer), recordings(recordings) {}
  virtual ~Sensor() = 0;
  virtual bool sendAlert() { return false; }
  virtual double getPrice() { return 0.0; }
  virtual std::string toString() = 0;
};

class TemperatureSensor : public Sensor {
 private:
  double diameter;
  double length;

 public:
  TemperatureSensor(std::string producer = "",
                    std::vector<double> recordings = {}, double diameter = 0.0,
                    double length = 0.0)
      : Sensor(producer, recordings), diameter(diameter), length(length){};
  ~TemperatureSensor() override{};
  bool sendAlert() override;
  double getPrice() override {
    if (diameter < 3 && length < 50) {
      return 17;
    }

    return 9;
  }
  std::string toString() override;
};

class HumiditySensor : public Sensor {
 public:
  HumiditySensor(std::string producer = "", std::vector<double> recordings = {})
      : Sensor(producer, recordings){};
  ~HumiditySensor() override{};
  bool sendAlert() override;
  double getPrice() override { return 4; }
  std::string toString() override;
};

class SmokeSensor : public Sensor {
 public:
  SmokeSensor(std::string producer = "", std::vector<double> recordings = {})
      : Sensor(producer, recordings){};
  ~SmokeSensor() override{};
  bool sendAlert() override;
  double getPrice() override { return 25; }
  std::string toString() override;
};