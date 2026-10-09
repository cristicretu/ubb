package model;

public class Bike implements Vehicle {
  String color;
  String registrationNumber;

  public Bike(String color, String registrationNumber) {
    this.color = color;
    this.registrationNumber = registrationNumber;
  }

  @Override
  public String getColor() {
    return color;
  }

  @Override
  public String getRegistrationNumber() {
    return registrationNumber;
  }

  @Override
  public String toString() {
    return "Bike [color=" + color + ", registrationNumber=" + registrationNumber + "]";
  }

}
