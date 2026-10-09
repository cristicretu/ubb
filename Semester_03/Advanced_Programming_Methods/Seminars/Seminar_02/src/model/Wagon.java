package model;

public class Wagon implements Vehicle {
  String color;
  String registrationNumber;

  public Wagon(String color, String registrationNumber) {
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
    return "Wagon [color=" + color + ", registrationNumber=" + registrationNumber + "]";
  }

}
