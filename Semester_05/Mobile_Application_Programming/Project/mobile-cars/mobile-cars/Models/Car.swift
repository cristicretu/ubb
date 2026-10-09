import Foundation

struct Car: Identifiable {
    let id: UUID
    let make: String
    let model: String
    let year: Int
    let licensePlate: String
    let currentMileage: Double
    let lastITPDate: Date
    let nextITPDate: Date
    let rovinietaExpiryDate: Date
    let carDescription: String?
}
