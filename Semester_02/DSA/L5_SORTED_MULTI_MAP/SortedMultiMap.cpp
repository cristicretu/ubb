#include "SortedMultiMap.h"

#include <exception>
#include <iostream>
#include <vector>

#include "SMMIterator.h"
using namespace std;

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)

*/
void SortedMultiMap::createNode(Node* node, TKey key, TValue value) {
  node->capacity = 1;
  node->size = 1;
  node->key = key;
  node->elems = new TValue[node->capacity];
  node->elems[0] = value;
  node->left = nullptr;
  node->right = nullptr;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)

*/
void SortedMultiMap::resizeNode(Node* node) {
  node->capacity *= 2;
  TValue* oldElems = node->elems;
  node->elems = new TValue[node->capacity];
  for (int i = 0; i < node->size; ++i) {
    node->elems[i] = oldElems[i];
  }
  delete[] oldElems;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)

*/
void SortedMultiMap::deleteNode(Node* node) {
  if (node != nullptr) {
    deleteNode(node->left);  /// first, delete his children
    deleteNode(node->right);
    delete[] node->elems;
    delete node;
  }
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)

*/
SortedMultiMap::SortedMultiMap(Relation r) : r(r) {
  this->root = nullptr;
  this->length = 0;
}

/*
BC: Theta(1)
WC: Theta(n) - when the tree is degenerated
AC: Theta(number of levels)
*/
void SortedMultiMap::add(TKey c, TValue v) {
  if (this->root == nullptr) {  /// empty tree, create root
    this->root = new Node;
    createNode(this->root, c, v);
    ++this->length;
    return;
  }

  Node* node = this->root;  /// go through the tree, in order to find the key
  while (node->key != c) {
    if (this->r(c, node->key)) {  /// left side
      if (node->left) {           /// if left child exists, go through it
        node = node->left;
        continue;
      } else {
        node->left = new Node;  /// we found the place to insert the key
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

/*
BC: Theta(1)
WC: Theta(n)
AC: Theta(number of levels)
*/
vector<TValue> SortedMultiMap::search(TKey c) const {
  Node* node = this->root;
  while (node != nullptr) {
    if (node->key == c) {
      return vector<TValue>(node->elems, node->elems + node->size);
    }

    if (this->r(c, node->key)) {
      node = node->left;
    } else {
      node = node->right;
    }
  }

  return vector<TValue>();  /// empty vector
}

/*
BC: Theta(1)
WC: Theta(n)
AC: Theta(number of levels)
*/
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
      Node* nextParent =
          node;  /// we need to find the smallest element in the right subtree
      Node* next = node->right;
      while (next->left != nullptr) {
        nextParent = next;
        next = next->left;
      }

      /// then we need to replace the current node with the smallest element
      node->key = next->key;
      delete[] node->elems;
      node->elems = next->elems;
      node->size = next->size;
      node->capacity = next->capacity;

      /// then we need to unlink that smallest element
      if (nextParent->left == next) {
        nextParent->left = next->right;
      } else {
        nextParent->right = next->right;
      }

      next->elems = nullptr;
      delete next;
    }
  }

  return true;
}

int SortedMultiMap::size() const { return this->length; }

bool SortedMultiMap::isEmpty() const { return this->length == 0; }

SMMIterator SortedMultiMap::iterator() const { return SMMIterator(*this); }

SortedMultiMap::~SortedMultiMap() { deleteNode(this->root); }