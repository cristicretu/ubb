import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { Observable } from 'rxjs';
import { SoftwareDeveloper } from '../models/software-developer.model';

@Injectable({
  providedIn: 'root',
})
export class SoftwareDeveloperService {
  private apiUrl = 'http://localhost:8080/softwareDeveloper';

  constructor(private http: HttpClient) {}

  getAllDevelopers(): Observable<SoftwareDeveloper[]> {
    const params = new HttpParams().set('action', 'readAll');
    return this.http.get<SoftwareDeveloper[]>(this.apiUrl, { params });
  }

  getDeveloperByName(name: string): Observable<SoftwareDeveloper> {
    const params = new HttpParams().set('action', 'findOne').set('name', name);
    return this.http.get<SoftwareDeveloper>(this.apiUrl, { params });
  }

  createDeveloper(
    developer: SoftwareDeveloper
  ): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('name', developer.name);
    formData.append('age', developer.age.toString());
    formData.append('skills', developer.skills);

    return this.http.post<{ success: boolean }>(this.apiUrl, formData);
  }

  updateDeveloper(
    developer: SoftwareDeveloper
  ): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('id', developer.id!.toString());
    formData.append('name', developer.name);
    formData.append('age', developer.age.toString());
    formData.append('skills', developer.skills);

    return this.http.put<{ success: boolean }>(this.apiUrl, formData);
  }

  deleteDeveloper(id: number): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('id', id.toString());

    return this.http.delete<{ success: boolean }>(this.apiUrl, {
      body: formData,
    });
  }
}
