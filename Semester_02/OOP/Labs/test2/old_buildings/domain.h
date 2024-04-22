#pragma once
#include <string>

class Building {
 protected:
  std::string address;
  int constructionYear;

 public:
  Building(std::string address = "", int constructionYear = 0)
      : address(address), constructionYear(constructionYear) {}
  virtual ~Building() = 0;
  virtual bool mustBeRestored() { return false; }
  virtual bool canBeDemolished() { return false; }
  virtual std::string toString() = 0;
  int getConstructionYear() { return constructionYear; }
};

class Block : public Building {
 private:
  int totalApartments;
  int occupiedApartments;

 public:
  Block(std::string address = "", int constructionYear = 0,
        int totalApartments = 0, int occupiedApartments = 0)
      : Building(address, constructionYear),
        totalApartments(totalApartments),
        occupiedApartments(occupiedApartments) {}
  ~Block() {}
  bool mustBeRestored() override;
  bool canBeDemolished() override {
    return occupiedApartments / totalApartments < 0.05;
  };
  std::string toString() override;
};

class House : public Building {
 private:
  bool isHistorical;

 public:
  House(std::string address = "", int constructionYear = 0,
        bool isHistorical = false)
      : Building(address, constructionYear), isHistorical(isHistorical) {}
  ~House() {}
  bool mustBeRestored() override;
  bool canBeDemolished() override { return !isHistorical; };
  std::string toString() override;
};
