// CarsViewModel.swift
// ViewModel for the car list (main screen)
// Manages the list of cars with @Published properties

import Foundation
import Combine
import SwiftUI

class CarsViewModel: ObservableObject {
    @Published var cars: [Car] = [
        Car(id: UUID(), make: "Toyota", model: "Corolla", year: 2020, licensePlate: "ABC123", currentMileage: 10000, lastITPDate: Date(), nextITPDate: Date(), rovinietaExpiryDate: Date(), carDescription: "Description"),
        Car(id: UUID(), make: "Ford", model: "Mustang", year: 2021, licensePlate: "XYZ789", currentMileage: 20000, lastITPDate: Date(), nextITPDate: Date(), rovinietaExpiryDate: Date(), carDescription: "Description"),
        Car(id: UUID(), make: "Chevrolet", model: "Camaro", year: 2022, licensePlate: "LMN456", currentMileage: 30000, lastITPDate: Date(), nextITPDate: Date(), rovinietaExpiryDate: Date(), carDescription: "Description"),
        Car(id: UUID(), make: "BMW", model: "X5", year: 2023, licensePlate: "PQR135", currentMileage: 40000, lastITPDate: Date(), nextITPDate: Date(), rovinietaExpiryDate: Date(), carDescription: "Description"),
        Car(id: UUID(), make: "Audi", model: "A4", year: 2024, licensePlate: "STU792", currentMileage: 50000, lastITPDate: Date(), nextITPDate: Date(), rovinietaExpiryDate: Date(), carDescription: "Description"),
    ]
    
    func addCar(_ car: Car) {
        cars.append(car)
    }
    
    func updateCar(_ car: Car) {
        if let index = cars.firstIndex(where: { $0.id == car.id }) {
            cars[index] = car
        }
    }
    
    func deleteCar(at offsets: IndexSet) {
        cars.remove(atOffsets: offsets)
    }
    
    func deleteCar(withId id: UUID) {
        cars.removeAll { $0.id == id }
    }
}
