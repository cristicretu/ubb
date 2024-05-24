#pragma once
#include <algorithm>

#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service(){};
  ~Service(){};

  std::vector<Document> getAll() const {
    std::vector<Document> documents = repo.getDocuments();

    std::sort(documents.begin(), documents.end(),
              [](const Document &a, const Document &b) {
                return a.getName() < b.getName();
              });

    return documents;
  }

  std::vector<Document> search(std::string nameOrKeyword) const {
    std::vector<Document> documents = repo.getDocuments();
    std::vector<Document> ans;

    std::copy_if(documents.begin(), documents.end(), ans.begin(),
                 [nameOrKeyword](const Document &a) {
                   if (a.getName().find(nameOrKeyword) != std::string::npos) {
                     return 1;
                   }

                   for (auto x : a.getKeywords()) {
                     if (x.find(nameOrKeyword) != std::string::npos) {
                       return 1;
                     }
                   }

                   return 0;
                 });

    return ans;
  }

  Document getBestMatchingDoc(std::string name) const {
    std::vector<Document> documents = repo.getDocuments();

    Document ans = documents[documents.size() - 1];
    documents.pop_back();
    auto getScore = [name](const Document &a) {
      return a.getName().find(name) != std::string::npos
                 ? name.size() / a.getName().size()
                 : 0;
    };

    double bestScore = getScore(ans);

    for (auto x : documents) {
      double score = getScore(x);
      if (score > bestScore) {
        bestScore = score;
        ans = x;
      }
    }

    return ans;
  }
};
