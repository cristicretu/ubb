#pragma once
#include "domain.h"
#include "repository.h"

class Service {
 private:
  Repository repo;

 public:
  Service();
  ~Service();
  bool addGene(std::string org_name, std::string gene_name,
               std::string gene_seq);
  bool removeGene(std::string org_name, std::string gene_name,
                  std::string gene_seq);

  std::vector<Gene> getAllGenes() const;
  std::vector<Gene> filterGenes(std::string gene_seq) const;
};