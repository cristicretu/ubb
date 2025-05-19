import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable, map } from "rxjs";
import { Car } from "../models/car.model";

@Injectable({
  providedIn: "root",
})
export class CarService {
  private apiBase = "http://localhost:8000/api";
  private apiUrl = `${this.apiBase}/cars`;

  constructor(private http: HttpClient) {}

  getCars(categoryId: number): Observable<{ records: Car[] }> {
    return this.http.get<{ records: Car[] }>(
      `${this.apiUrl}/read?category_id=${categoryId}`
    );
  }

  getCar(id: number): Observable<{ data: Car }> {
    return this.http
      .get<{ success: boolean; record: Car }>(
        `${this.apiUrl}/read_one?id=${id}`
      )
      .pipe(map((response) => ({ data: response.record })));
  }

  createCar(
    car: Car
  ): Observable<{ success: boolean; message: string; id?: number }> {
    const carForApi = {
      Id: car.id,
      Model: car.model,
      EnginePower: car.engine_power,
      FuelType: car.fuel_type,
      Color: car.color,
      Year: car.year,
      Price: car.price,
      Features: car.features || null,
      CategoryId: car.category_id,
    };

    return this.http.post<{ success: boolean; message: string; id?: number }>(
      `${this.apiUrl}/create`,
      carForApi
    );
  }

  updateCar(car: Car): Observable<{ success: boolean; message: string }> {
    const carForApi = {
      Id: car.id,
      Model: car.model,
      EnginePower: car.engine_power,
      FuelType: car.fuel_type,
      Color: car.color,
      Year: car.year,
      Price: car.price,
      Features: car.features || null,
      CategoryId: car.category_id,
    };

    return this.http.post<{ success: boolean; message: string }>(
      `${this.apiUrl}/edit`,
      carForApi
    );
  }

  deleteCar(id: number): Observable<{ success: boolean; message: string }> {
    return this.http.delete<{ success: boolean; message: string }>(
      `${this.apiUrl}/delete?id=${id}`
    );
  }
}
