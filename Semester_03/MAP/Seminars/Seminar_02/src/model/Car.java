package model;

public class Car implements Vehicle {
  String color;

  public Car(String color) {
    this.color = color;
  }

  @Override
  public String getColor() {
    return color;
  }

}
