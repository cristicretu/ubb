package ui;

import service.service;
import java.util.Scanner;

import model.Bike;
import model.Car;
import model.Vehicle;
import model.Wagon;
import repository.capacityExceededException;

public class ui {
  private service service;

  public ui(service service) {
    this.service = service;
  }

  public void printMenu() {
    System.out.println("1. Add vehicle");
    System.out.println("2. Get all vehicles");
    System.out.println("3. Get all red vehicles");
    System.out.println("4. Exit");
  }

  public void addVehicle(Scanner scanner) {
    System.out.println("Enter vehicle type (car, wagon, bike):");
    String type = scanner.nextLine();
    System.out.println("Enter vehicle color:");
    String color = scanner.nextLine();
    System.out.println("Enter vehicle registration number:");
    String registrationNumber = scanner.nextLine();

    Vehicle vehicle = null;
    switch (type) {
      case "car":
        vehicle = new Car(color, registrationNumber);
        break;
      case "wagon":
        vehicle = new Wagon(color, registrationNumber);
        break;
      case "bike":
        vehicle = new Bike(color, registrationNumber);
        break;
      default:
        System.out.println("Invalid vehicle type");
        return;
    }

    try {
      service.add(vehicle);
    } catch (capacityExceededException e) {
      System.out.println("Capacity exceeded");
    }
  }

  public void getAllVehicles() {
    Vehicle[] vehicles = service.getAll();
    for (Vehicle vehicle : vehicles) {
      System.out.println(vehicle);
    }
  }

  public void getAllRedVehicles() {
    Vehicle[] vehicles = service.getAll();
    for (Vehicle vehicle : vehicles) {
      if (vehicle.getColor().equals("red")) {
        System.out.println(vehicle);
      }
    }
  }

  public void populate6Vehicles() {
    try {
      service.add(new Car("red", "B123456"));
      service.add(new Car("blue", "M123457"));
      service.add(new Bike("green", "B123458"));
      service.add(new Wagon("yellow", "M123459"));
      service.add(new Wagon("red", "M123460"));
      service.add(new Bike("red", "B123461"));
    } catch (capacityExceededException e) {
      System.out.println("Capacity exceeded");
    }
  }

  public void run() {
    Scanner scanner = new Scanner(System.in);
    populate6Vehicles();
    int option = 0;
    do {
      printMenu();
      option = scanner.nextInt();
      scanner.nextLine();

      switch (option) {
        case 1:
          addVehicle(scanner);
          break;
        case 2:
          getAllVehicles();
          break;
        case 3:
          getAllRedVehicles();
          break;
      }
    } while (option != 4);

  }
}
