import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable } from "rxjs";
import { Car } from "../models/car.model";

@Injectable({
  providedIn: "root",
})
export class CarService {
  private apiUrl = "/api/cars";

  constructor(private http: HttpClient) {}

  getCars(categoryId: number): Observable<{ records: Car[] }> {
    return this.http.get<{ records: Car[] }>(
      `${this.apiUrl}/read.php?category_id=${categoryId}`
    );
  }

  getCar(id: number): Observable<{ data: Car }> {
    return this.http.get<{ data: Car }>(`${this.apiUrl}/read_one.php?id=${id}`);
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
