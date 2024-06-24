#pragma once
#include <algorithm>
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
  vector<tuple<int, string, int>> getOffers() const {
    auto ofr = offers;
    sort(
        ofr.begin(), ofr.end(),
        [](const tuple<int, string, int>& a, const tuple<int, string, int>& b) {
          return get<2>(a) < get<2>(b);
        });
    return ofr;
  }

  void setPrice(int price) { this->price = price; }

  void addOffer(int id, string date, int price) {
    offers.emplace_back(make_tuple(id, date, price));
  }

  string toString() const {
    string result = name + " | " + category + " | " + to_string(price) + " | ";
    for (const auto& offer : offers) {
      result += to_string(get<0>(offer)) + ", " + get<1>(offer) + ", " +
                to_string(get<2>(offer)) + " | ";
    }
    return result;
  }
};
