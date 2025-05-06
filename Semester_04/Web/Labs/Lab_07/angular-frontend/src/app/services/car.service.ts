import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable, map } from "rxjs";
import { Car } from "../models/car.model";

@Injectable({
  providedIn: "root",
})
export class CarService {
  // Always use the direct URL to the backend
  private apiBase = "http://localhost:8000/api";
  private apiUrl = `${this.apiBase}/cars`;

  constructor(private http: HttpClient) {}

  getCars(categoryId: number): Observable<{ records: Car[] }> {
    return this.http.get<{ records: Car[] }>(
      `${this.apiUrl}/read.php?category_id=${categoryId}`
    );
  }

  getCar(id: number): Observable<{ data: Car }> {
    return this.http
      .get<{ success: boolean; record: Car }>(
        `${this.apiUrl}/read_one.php?id=${id}`
      )
      .pipe(map((response) => ({ data: response.record })));
  }

  createCar(
    car: Car
  ): Observable<{ success: boolean; message: string; id?: number }> {
    return this.http.post<{ success: boolean; message: string; id?: number }>(
      `${this.apiUrl}/create.php`,
      car
    );
  }

  updateCar(car: Car): Observable<{ success: boolean; message: string }> {
    return this.http.post<{ success: boolean; message: string }>(
      `${this.apiUrl}/edit.php`,
      car
    );
  }

  deleteCar(id: number): Observable<{ success: boolean; message: string }> {
    return this.http.delete<{ success: boolean; message: string }>(
      `${this.apiUrl}/delete.php?id=${id}`
    );
  }
}
