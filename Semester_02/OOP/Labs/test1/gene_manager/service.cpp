#include "service.h"

#include <algorithm>

Service::Service() { this->repo = Repository(); }

Service::~Service() {}

bool Service::addGene(std::string org_name, std::string gene_name,
                      std::string gene_seq) {
  Gene gene = Gene(org_name, gene_name, gene_seq);
  if (this->repo.addGene(gene)) {
    return true;
  }
  throw std::runtime_error("Gene already exists!");
}

bool Service::removeGene(std::string org_name, std::string gene_name,
                         std::string gene_seq) {
  Gene gene = Gene(org_name, gene_name, gene_seq);
  if (this->repo.removeGene(gene)) {
    return true;
  }
  throw std::runtime_error("Gene does not exist!");
}

std::vector<Gene> Service::getAllGenes() const {
  std::vector<Gene> genes = this->repo.getGenes();
  sort(genes.begin(), genes.end(), [](Gene a, Gene b) { return a > b; });
  return genes;
}

std::vector<Gene> Service::filterGenes(std::string gene_seq) const {
  std::vector<Gene> genes = this->repo.getGenes();
  std::vector<Gene> filtered_genes;
  for (Gene gene : genes) {
    if (gene.get_gene_seq().find(gene_seq) != std::string::npos) {
      filtered_genes.push_back(gene);
    }
  }
  return filtered_genes;
}