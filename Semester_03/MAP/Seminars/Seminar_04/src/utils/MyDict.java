package utils;

import java.util.Map;
import java.util.HashMap;

public class MyDict<K, V> implements IDict<K, V> {
  private Map<K, V> dict;

  public MyDict() {
    dict = new HashMap<K, V>();
  }

  @Override
  public void put(K key, V value) {
    dict.put(key, value);
  }

  @Override
  public V get(K key) {
    return dict.get(key);
  }

  @Override
  public boolean isDefined(K key) {
    return dict.containsKey(key);
  }
}
