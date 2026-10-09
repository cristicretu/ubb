#pragma once
#include <iostream>

template <typename T>
class DynArray {
 private:
  int size, capacity;
  T *elems;

  void resize();

 public:
  DynArray(int capacity = 5);

  DynArray(const DynArray<T> &array);

  ~DynArray();

  DynArray<T> &operator=(const DynArray<T> &array);

  T &operator[](int index);
  const T &operator[](int index) const;

  void add(const T &elem);
  void remove(int index);
  void set(int index, const T &elem);
  int getSize() const;
  int getCapacity() const;

 public:
  class Iterator {
   private:
    T *current = nullptr;
    int index = 0;

   public:
    Iterator(T *current, int index = 0) : current(current), index(index) {}

    Iterator() = default;

    T &operator*() { return *current; }

    Iterator &operator++() {
      current++;
      index++;
      return *this;
    }

    Iterator operator++(int) {
      Iterator copy = *this;
      ++(*this);
      return copy;
    }

    bool operator!=(const Iterator &other) const {
      return current != other.current;
    }
    bool operator==(const Iterator &other) const {
      return current == other.current;
    }

    int getIndex() const { return index; }
  };

  Iterator begin() { return Iterator(elems, 0); }

  Iterator end() { return Iterator(elems + size, size); }
};

template <typename T>
void DynArray<T>::resize() {
  capacity *= 2;
  T *newElems = new T[capacity];
  for (int i = 0; i < size; i++) {
    newElems[i] = elems[i];
  }

  delete[] elems;
  elems = newElems;
}

template <typename T>
DynArray<T>::DynArray(int capacity)
    : capacity(capacity), size(0), elems(new T[capacity]) {}

template <typename T>
DynArray<T>::DynArray(const DynArray<T> &array)
    : capacity(array.capacity), size(array.size), elems(new T[capacity]) {
  for (int i = 0; i < size; i++) {
    elems[i] = array.elems[i];
  }
}

template <typename T>
DynArray<T>::~DynArray() {
  delete[] elems;
}

template <typename T>
DynArray<T> &DynArray<T>::operator=(const DynArray<T> &array) {
  if (this != &array) {
    delete[] elems;
    capacity = array.capacity;
    size = array.size;
    elems = new T[capacity];
    for (int i = 0; i < size; i++) {
      elems[i] = array.elems[i];
    }
  }
  return *this;
}

template <typename T>
T &DynArray<T>::operator[](int index) {
  return elems[index];
}

template <typename T>
const T &DynArray<T>::operator[](int index) const {
  return elems[index];
}

template <typename T>
void DynArray<T>::add(const T &elem) {
  if (size == capacity) {
    resize();
  }
  elems[size++] = elem;
}

template <typename T>
void DynArray<T>::remove(int index) {
  for (int i = index; i < size - 1; i++) {
    elems[i] = elems[i + 1];
  }
  size--;
}

template <typename T>
void DynArray<T>::set(int index, const T &elem) {
  elems[index] = elem;
}

template <typename T>
int DynArray<T>::getSize() const {
  return size;
}

template <typename T>
int DynArray<T>::getCapacity() const {
  return capacity;
}
