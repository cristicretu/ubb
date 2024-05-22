#include "SortedMultiMap.h"

#include <exception>
#include <iostream>
#include <vector>

#include "SMMIterator.h"
using namespace std;

void SortedMultiMap::createNode(Node* node, TKey key, TValue value) {
  node->capacity = 1;
  node->size = 1;
  node->key = key;
  node->elems = new TValue[node->capacity];
  node->elems[0] = value;
  node->left = nullptr;
  node->right = nullptr;
}

void SortedMultiMap::resizeNode(Node* node) {
  node->capacity *= 2;
  TValue* oldElems = node->elems;
  node->elems = new TValue[node->capacity];
  for (int i = 0; i < node->size; ++i) {
    node->elems[i] = oldElems[i];
  }
  delete[] oldElems;
}

void SortedMultiMap::deleteNode(Node* node) {
  if (node != nullptr) {
    deleteNode(node->left);
    deleteNode(node->right);
    delete[] node->elems;
    delete node;
  }
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
    resizeNode(node);
  }

  node->elems[node->size++] = v;
  ++this->length;

  return;
}

vector<TValue> SortedMultiMap::search(TKey c) const {
  vector<TValue> ans;

  Node* node = this->root;
  while (node != nullptr) {
    if (node->key == c) {
      for (int i = 0; i < node->size; ++i) {
        ans.push_back(node->elems[i]);
      }
      return ans;
    }

    if (this->r(c, node->key)) {
      node = node->left;
    } else {
      node = node->right;
    }
  }

  return ans;
}

bool SortedMultiMap::remove(TKey c, TValue v) {
  Node* parent = nullptr;
  Node* node = this->root;
  while (node != nullptr && node->key != c) {
    parent = node;
    if (this->r(c, node->key)) {
      node = node->left;
    } else {
      node = node->right;
    }
  }

  if (node == nullptr) {  /// node does not exist
    return false;
  }

  int index = -1;
  for (int i = 0; i < node->size; ++i) {
    if (node->elems[i] == v) {
      index = i;
      break;
    }
  }

  if (index == -1) {  /// value does not exist
    return false;
  }

  for (int i = index; i < node->size - 1; ++i) {
    node->elems[i] = node->elems[i + 1];
  }
  --node->size;
  --this->length;

  if (node->size == 0) {
    if (node->left == nullptr && node->right == nullptr) {  /// no children
      if (parent == nullptr) {
        deleteNode(node);
        this->root = nullptr;
      } else {  /// unlink node
        if (parent->left == node) {
          parent->left = nullptr;
        } else {
          parent->right = nullptr;
        }
        deleteNode(node);
      }
    } else if (node->left == nullptr || node->right == nullptr) {  /// one child
      Node* child = (node->left != nullptr) ? node->left : node->right;
      if (parent == nullptr) {
        this->root = child;
      } else {
        if (parent->left == node) {
          parent->left = child;
        } else {
          parent->right = child;
        }
      }
      delete[] node->elems;
      delete node;
    } else {  /// two children
      Node* successorParent = node;
      Node* successor = node->right;
      while (successor->left != nullptr) {
        successorParent = successor;
        successor = successor->left;
      }

      node->key = successor->key;
      delete[] node->elems;
      node->elems = successor->elems;
      node->size = successor->size;
      node->capacity = successor->capacity;

      if (successorParent->left == successor) {
        successorParent->left = successor->right;
      } else {
        successorParent->right = successor->right;
      }

      successor->elems = nullptr;
      delete successor;
    }
  }

  return true;
}

int SortedMultiMap::size() const { return this->length; }

bool SortedMultiMap::isEmpty() const { return this->length == 0; }

SMMIterator SortedMultiMap::iterator() const { return SMMIterator(*this); }

SortedMultiMap::~SortedMultiMap() { deleteNode(this->root); }