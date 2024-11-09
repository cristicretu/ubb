package utils;

import java.util.List;

import exceptions.ListException;

import java.util.ArrayList;

public class MyList<T> implements IList<T> {
  private List<T> list;

  public MyList() {
    list = new ArrayList<T>();
  }

  @Override
  public void add(T item) {
    list.add(item);
  }

  @Override
  public T get(int index) throws ListException {
    if (index < 0 || index >= size()) {
      throw new ListException("Index out of bounds: " + index);
    }
    return list.get(index);
  }

  private int size() {
    return list.size();
  }

  @Override
  public String toString() {
    return list.toString();
  }
}
