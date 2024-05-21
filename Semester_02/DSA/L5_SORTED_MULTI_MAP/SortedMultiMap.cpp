#include "SortedMultiMap.h"

#include <exception>
#include <iostream>
#include <vector>

#include "SMMIterator.h"
using namespace std;

void SortedMultiMap::createNode(Node*& node, TKey key, TValue value) {
  node->capacity = 1;
  node->size = 1;

  node->key = key;
  node->elems = new TValue[node->capacity], node->elems[0] = value;
  node->left = nullptr;
  node->right = nullptr;
}

SortedMultiMap::SortedMultiMap(Relation r) : r(r) {
  this->root = nullptr;
  this->length = 0;
}

void SortedMultiMap::add(TKey c, TValue v) {
  if (this->root == nullptr) {
    this->root = new Node;
    createNode(this->root, c, v);
    ++this->length;
    return;
  }

  Node* node = this->root;
  while (node->key != c) {
    if (this->r(c, node->key)) {  /// left side
      if (node->left) {
        node = node->left;
        continue;
      } else {
        node->left = new Node;
        createNode(node->left, c, v);
        ++this->length;
        return;
      }
    } else {  /// right side
      if (node->right) {
        node = node->right;
        continue;
      } else {
        node->right = new Node;
        createNode(node->right, c, v);
        ++this->length;
        return;
      }
    }
  }

  /// check if it's full
  if (node->size == node->capacity) {
    /// resize
  }

  node->elems[node->size++] = v;
  ++this->length;

  return;
}

vector<TValue> SortedMultiMap::search(TKey c) const {
  // TODO - Implementation
  return vector<TValue>();
}

bool SortedMultiMap::remove(TKey c, TValue v) {
  // TODO - Implementation
  return false;
}

int SortedMultiMap::size() const { return this->length; }

bool SortedMultiMap::isEmpty() const { return this->length == 0; }

SMMIterator SortedMultiMap::iterator() const { return SMMIterator(*this); }

SortedMultiMap::~SortedMultiMap() {
  std::function<void(Node*)> deleteNode = [&](Node* node) {
    if (node != nullptr) {
      deleteNode(node->left);
      deleteNode(node->right);
      delete node;
    }
  };

  deleteNode(this->root);
}