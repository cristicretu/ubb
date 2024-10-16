package service;

import model.Vehicle;
import repository.repository;
import repository.capacityExceededException;

public class service {
  private repository repository;

  public service(repository repository) {
    this.repository = repository;
  }

  public void add(Vehicle vehicle) throws capacityExceededException {
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

  public Vehicle findVehicleByRegistrationNumber(String registrationNumber) throws vehicleNotFoundException {
    Vehicle[] vehicles = this.repository.getAll();
    for (Vehicle vehicle : vehicles) {
      if (vehicle != null && vehicle.getRegistrationNumber().equals(registrationNumber)) {
        return vehicle;
      }
    }
    throw new vehicleNotFoundException("Vehicle not found");
  }

  public Vehicle[] getByColor(String color) {
    Vehicle[] vehicles = this.repository.getAll();
    Vehicle[] filteredVehicles = new Vehicle[vehicles.length];
    int index = 0;
    for (Vehicle vehicle : vehicles) {
      if (vehicle != null && vehicle.getColor().equals(color)) {
        filteredVehicles[index++] = vehicle;
      }
    }
    return filteredVehicles;
  }
}
