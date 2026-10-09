import { Injectable, inject } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import { DevelopersResponse } from '../models/developer.model';

@Injectable({
  providedIn: 'root',
})
export class DeveloperService {
  private http = inject(HttpClient);
  private baseUrl = 'http://localhost:3000';

  getDevelopers(skill?: string): Observable<DevelopersResponse> {
    const url = skill
      ? `${this.baseUrl}/api/developers?skill=${encodeURIComponent(skill)}`
      : `${this.baseUrl}/api/developers`;

    return this.http.get<DevelopersResponse>(url, {
      withCredentials: true,
    });
  }
}
