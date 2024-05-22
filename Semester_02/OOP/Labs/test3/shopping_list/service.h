#pragma once

#include <algorithm>

#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service() : repo("../items.txt") {}
  ~Service(){};

  std::vector<Item> getItems() {
    std::vector<Item> items = repo.getItems();

    std::sort(items.begin(), items.end(), [](const Item &a, const Item &b) {
      return a.getCategory() == b.getCategory()
                 ? a.getName() < b.getName()
                 : a.getCategory() < b.getCategory();
    });

    return items;
  }

  std::vector<Item> search(std::string nameOrCategory) {
    std::vector<Item> items = repo.getItems();
    std::vector<Item> ans(items.size());

    auto filtered = std::copy_if(items.begin(), items.end(), ans.begin(),
                                 [](auto &elem) { return true; });

    ans.resize(std::distance(ans.begin(), filtered));

    return ans;
  }
};