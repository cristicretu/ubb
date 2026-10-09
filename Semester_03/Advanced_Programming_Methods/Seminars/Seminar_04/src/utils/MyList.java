package utils;

import java.util.List;
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
  public T get(int index) {
    return list.get(index);
  }

  @Override
  public String toString() {
    return list.toString();
  }
}
