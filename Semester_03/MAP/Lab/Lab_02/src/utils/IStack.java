package utils;

import java.util.List;

public interface IStack<T> {
  void push(T item);

  T pop();

  T top();

  boolean isEmpty();

  int size();

  List<T> getList();
}
