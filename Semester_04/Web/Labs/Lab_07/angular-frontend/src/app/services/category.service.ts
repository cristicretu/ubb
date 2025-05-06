import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable } from "rxjs";
import { Category } from "../models/category.model";

@Injectable({
  providedIn: "root",
})
export class CategoryService {
  private apiUrl = "/api/categories.php";

  constructor(private http: HttpClient) {}

  getCategories(): Observable<{ records: Category[] }> {
    return this.http.get<{ records: Category[] }>(this.apiUrl);
  }
}
