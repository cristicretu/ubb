package service;

import model.Vehicle;
import repository.repository;

public class service {
  private repository repository;

  public service(repository repository) {
    this.repository = repository;
  }

  public void add(Vehicle vehicle) {
    this.repository.add(vehicle);
  }

  public void remove(Vehicle vehicle) {
    this.repository.remove(vehicle);
  }

  public void update(Vehicle vehicle) {
    this.repository.update(vehicle);
  }

  public Vehicle[] getAll() {
    return this.repository.getAll();
  }

}
