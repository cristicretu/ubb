package utils;

public interface IDict<K, V> {
  void put(K key, V value);

  V get(K key);

  boolean isDefined(K key);
}
