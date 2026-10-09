// CarFormView.swift
// Form view for creating and editing cars
// Pre-populated for edit, empty for create
// Includes validation and error handling

import SwiftUI

struct CarFormView: View {
    let car: Car?
    let onSave: (Car) -> Void
    
    @State private var make = ""
    @State private var model = ""
    @State private var year = ""
    @State private var licensePlate = ""
    @State private var currentMileage = ""
    @State private var lastITPDate = Date()
    @State private var nextITPDate = Date()
    @State private var rovinietaExpiryDate = Date()
    @State private var carDescription = ""
    @State private var showingError = false
    @State private var errorMessage = ""

    @Environment(\.dismiss) private var dismiss
    
    init(car: Car? = nil, onSave: @escaping (Car) -> Void) {
        self.car = car
        self.onSave = onSave
        _make = State(initialValue: car?.make ?? "")
        _model = State(initialValue: car?.model ?? "")
        _year = State(initialValue: car.map { String($0.year) } ?? "")
        _licensePlate = State(initialValue: car?.licensePlate ?? "")
        _currentMileage = State(initialValue: car.map { String($0.currentMileage) } ?? "")
        _lastITPDate = State(initialValue: car?.lastITPDate ?? Date())
        _nextITPDate = State(initialValue: car?.nextITPDate ?? Date())
        _rovinietaExpiryDate = State(initialValue: car?.rovinietaExpiryDate ?? Date())
        _carDescription = State(initialValue: car?.carDescription ?? "")
    }
    
    private func saveCar() {
        guard !make.isEmpty else {
            errorMessage = "Make is required"
            showingError = true
            return
        }
        
        guard !model.isEmpty else {
            errorMessage = "Model is required"
            showingError = true
            return
        }
        
        guard let yearInt = Int(year), yearInt > 1900, yearInt <= 2100 else {
            errorMessage = "Please enter a valid year"
            showingError = true
            return
        }
        
        guard !licensePlate.isEmpty else {
            errorMessage = "License plate is required"
            showingError = true
            return
        }
        
        guard let mileageDouble = Double(currentMileage), mileageDouble >= 0 else {
            errorMessage = "Please enter a valid mileage"
            showingError = true
            return
        }
        
        let savedCar = Car(
            id: car?.id ?? UUID(),
            make: make,
            model: model,
            year: yearInt,
            licensePlate: licensePlate,
            currentMileage: mileageDouble,
            lastITPDate: lastITPDate,
            nextITPDate: nextITPDate,
            rovinietaExpiryDate: rovinietaExpiryDate,
            carDescription: carDescription.isEmpty ? nil : carDescription
        )
        
        onSave(savedCar)
        dismiss()
    }

    var body: some View {
      Form {
        Section(header: Text("Car Information")) {
          TextField("Make", text: $make)
          TextField("Model", text: $model)
          TextField("Year", text: $year)
            .keyboardType(.numberPad)
          TextField("License Plate", text: $licensePlate)
          TextField("Current Mileage", text: $currentMileage)
            .keyboardType(.decimalPad)
          DatePicker("Last ITP Date", selection: $lastITPDate, displayedComponents: .date)
          DatePicker("Next ITP Date", selection: $nextITPDate, displayedComponents: .date)
          DatePicker("Rovinieta Expiry Date", selection: $rovinietaExpiryDate, displayedComponents: .date)
          TextField("Car Description", text: $carDescription)
        }
        Section {
          Button(action: saveCar) {
            Text("Save")
              .frame(maxWidth: .infinity)
          }
          .buttonStyle(.borderedProminent)
        }
      }
      .navigationTitle(car == nil ? "Add Car" : "Edit Car")
      .navigationBarTitleDisplayMode(.inline)
      .toolbar {
        ToolbarItem(placement: .navigationBarLeading) {
          Button("Cancel") {
            dismiss()
          }
        }
        ToolbarItem(placement: .navigationBarTrailing) {
          Button("Save") {
            saveCar()
          }
        }
      }
      .alert("Validation Error", isPresented: $showingError) {
        Button("OK", role: .cancel) { }
      } message: {
        Text(errorMessage)
      }
    }
}