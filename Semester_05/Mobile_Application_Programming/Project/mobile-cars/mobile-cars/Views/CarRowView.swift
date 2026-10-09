// CarRowView.swift
// Individual row/cell in the car list
// Shows basic car info with deadline indicators

import SwiftUI

struct CarRowView: View {
    let car: Car

    var body: some View {
        HStack {
          VStack(alignment: .leading) {
            HStack {
            Text(car.make)
            Text(car.model)
            }
            Text(car.licensePlate).font(.subheadline).foregroundColor(.secondary)
          }

          Spacer()

          Image(systemName: "chevron.right")
        }
    }
}