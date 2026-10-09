package model;

public class Car implements Vehicle {
  String color;
  String registrationNumber;

  public Car(String color, String registrationNumber) {
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
    return "Car [color=" + color + ", registrationNumber=" + registrationNumber + "]";
  }

}
