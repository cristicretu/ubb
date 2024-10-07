package repository;

import model.Vehicle;

public class inMemoryRepo implements repository {
  private Vehicle[] vehicles;
  private int size;
  private int capacity;

  public inMemoryRepo() {
    this.vehicles = new Vehicle[10];
    this.size = 0;
    this.capacity = 10;
  }

  public inMemoryRepo(int size) {
    this.vehicles = new Vehicle[size];
    this.size = 0;
    this.capacity = size;
  }

  @Override
  public void add(Vehicle vehicle) {
    if (this.size == this.capacity) {
      Vehicle[] newVehicles = new Vehicle[this.capacity * 2];
      for (int i = 0; i < this.size; i++) {
        newVehicles[i] = this.vehicles[i];
      }
      this.vehicles = newVehicles;
    }
    this.vehicles[this.size] = vehicle;
    this.size++;
  }

  @Override
  public void remove(Vehicle vehicle) {
    for (int i = 0; i < this.size; i++) {
      if (this.vehicles[i] == vehicle) {
        this.vehicles[i] = null;
        this.size--;
        break;
      }
    }
  }

  @Override
  public void update(Vehicle vehicle) {
    for (int i = 0; i < this.size; i++) {
      if (this.vehicles[i] == vehicle) {
        this.vehicles[i] = vehicle;
        break;
      }
    }
  }

  @Override
  public Vehicle[] getAll() {
    return this.vehicles;
  }

}
