package utils;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import exceptions.HeapException;
import model.value.IValue;

public class MyHeap<K, V> implements IHeap<K, V> {
  private Integer firstFreeAddress;
  private Map<K, V> heap;

  public MyHeap() {
    this.firstFreeAddress = 1;
    this.heap = new HashMap<>();
  }

  @Override
  public V get(K key) throws HeapException {
    if (!this.isDefined(key)) {
      throw new HeapException("Key not found in heap");
    }
    return this.heap.get(key);
  }

  @Override
  public void put(K key, V value) {
    this.heap.put(key, value);
  }

  @Override
  public boolean isDefined(K key) {
    return this.heap.containsKey(key);
  }

  @Override
  public void remove(K key) {
    this.heap.remove(key);
  }

  @Override
  public String toString() {
    if (this.heap.isEmpty()) {
      return "(the heap is empty)\n";
    }

    StringBuilder s = new StringBuilder();
    for (K key : this.heap.keySet()) {
      s.append(key.toString()).append(" -> ");
      s.append(this.heap.get(key).toString());
      s.append("\n");
    }
    return s.toString();
  }

  @Override
  public Map<K, V> getHeap() {
    return this.heap;
  }

  @Override
  public List<V> getValues() {
    return new LinkedList<V>(this.heap.values());
  }

  @Override
  public Map<Integer, IValue> safeGarbageCollector(Set<Integer> usedAddresses, Map<Integer, IValue> heap) {
    Map<Integer, IValue> newHeap = new HashMap<Integer, IValue>();
    for (Integer key : heap.keySet()) {
      if (usedAddresses.contains(key)) {
        newHeap.put(key, heap.get(key));
      }
    }
    return newHeap;
  }

  @Override
  public void setHeap(Map<K, V> heap) {
    this.heap = heap;
  }

  @Override
  public Integer allocate() {
    return this.firstFreeAddress++;
  }

}