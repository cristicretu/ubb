#pragma once
#include <iostream>
#include <string>
#include <vector>
using namespace std;

class User {
 private:
  int id;
  string name, type;  // collector, admin

 public:
  User(string name, int id, string type) : name(name), id(id), type(type){};

  string getName() const { return name; }
  int getId() const { return id; }
  string getType() const { return type; }
};

class Item {
 private:
  string name, category;
  int price;
  vector<tuple<int, string, int>> offers;

 public:
  Item(string name, string category, int price,
       vector<tuple<int, string, int>> offers = {})
      : name(name), category(category), price(price), offers(offers){};

  string getName() const { return name; }
  string getCategory() const { return category; }
  int getPrice() const { return price; }
  vector<tuple<int, string, int>> getOffers() const { return offers; }

  void addOffer(int id, string date, int price) {
    offers.emplace_back(make_tuple(id, date, price));
  }
};