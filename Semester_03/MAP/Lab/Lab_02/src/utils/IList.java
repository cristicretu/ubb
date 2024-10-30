package utils;

import exceptions.ListException;

public interface IList<T> {
  void add(T item);

  T get(int index) throws ListException;
}
