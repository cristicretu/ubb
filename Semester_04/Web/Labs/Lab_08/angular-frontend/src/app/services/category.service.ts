import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable } from "rxjs";
import { Category } from "../models/category.model";
import { environment } from "../../environments/environment";

@Injectable({
  providedIn: "root",
})
export class CategoryService {
  private apiUrl = environment.production
    ? "/api/categories"
    : "http://localhost:8000/api/categories";

  constructor(private http: HttpClient) {}

  getCategories(): Observable<{ records: Category[] }> {
    return this.http.get<{ records: Category[] }>(this.apiUrl);
  }
}
