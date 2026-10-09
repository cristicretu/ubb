import SwiftUI

struct CarListView: View {
    @StateObject private var viewModel = CarsViewModel()
    @State private var selectedCar: Car?
    @State private var showingForm = false
    @State private var carToDelete: Car?
    @State private var showingDeleteConfirmation = false

    var body: some View {
        NavigationStack {
            List {
                ForEach(viewModel.cars) { car in
                    CarRowView(car: car)
                        .onTapGesture {
                            selectedCar = car
                        }
                        .swipeActions(edge: .trailing, allowsFullSwipe: false) {
                            Button(role: .destructive) {
                                carToDelete = car
                                showingDeleteConfirmation = true
                            } label: {
                                Label("Delete", systemImage: "trash")
                            }
                        }
                }
                .onDelete { indexSet in
                    carToDelete = indexSet.first.map { viewModel.cars[$0] }
                    showingDeleteConfirmation = true
                }
            }
            .navigationTitle("My Cars")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button(action: {
                        showingForm = true
                    }) {
                        Image(systemName: "plus")
                    }
                }
            }
            .sheet(item: $selectedCar) { car in
                NavigationStack {
                    CarFormView(car: car) { updatedCar in
                        viewModel.updateCar(updatedCar)
                    }
                }
            }
            .sheet(isPresented: $showingForm) {
                NavigationStack {
                    CarFormView { newCar in
                        viewModel.addCar(newCar)
                    }
                }
            }
            .confirmationDialog(
                "Delete Car",
                isPresented: $showingDeleteConfirmation,
                titleVisibility: .visible
            ) {
                Button("Delete", role: .destructive) {
                    if let car = carToDelete {
                        viewModel.deleteCar(withId: car.id)
                    }
                }
                Button("Cancel", role: .cancel) {
                    carToDelete = nil
                }
            } message: {
                if let car = carToDelete {
                    Text("Are you sure you want to delete \(car.make) \(car.model)?")
                }
            }
        }
    }
}
