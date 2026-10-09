#pragma once

#include "SortedMultiMap.h"

class NodeStack {
 private:
  Node** data;
  int top, capacity;

  void resize() {
    capacity *= 2;
    Node** newData = new Node*[capacity];
    for (int i = 0; i < top + 1; ++i) {
      newData[i] = data[i];
    }
    delete[] data;
    data = newData;
  }

 public:
  NodeStack() : top(-1), capacity(10) { data = new Node*[capacity]; }

  ~NodeStack() { delete[] data; }

  void push(Node* node) {
    if (top == capacity - 1) {
      resize();
    }
    data[++top] = node;
  }

  Node* pop() {
    if (top == -1) {
      return nullptr;
    }
    return data[top--];
  }

  Node* peek() {
    if (top == -1) {
      return nullptr;
    }
    return data[top];
  }

  bool empty() const { return top == -1; }
};

class SMMIterator {
  friend class SortedMultiMap;

 private:
  // DO NOT CHANGE THIS PART
  const SortedMultiMap& map;
  SMMIterator(const SortedMultiMap& map);

  TElem* stack;
  int currentValueIndex;
  NodeStack st;

  // TODO - Representation

 public:
  void first();
  void next();
  bool valid() const;
  TElem getCurrent() const;

  // destructor
  ~SMMIterator();
};
