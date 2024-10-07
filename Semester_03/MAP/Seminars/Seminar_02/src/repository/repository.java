package repository;

import model.Vehicle;

public interface repository {

  public void add(Vehicle vehicle);

  public void remove(Vehicle vehicle);

  public void update(Vehicle vehicle);

  public Vehicle[] getAll();

}
