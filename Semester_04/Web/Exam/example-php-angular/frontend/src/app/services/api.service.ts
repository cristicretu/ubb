import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

export interface Developer {
  id: number;
  name: string;
  skills: string;
}

export interface Project {
  id: number;
  name: string;
  description?: string;
  members?: string;
  projectManagerID?: number;
}

export interface ApiResponse<T> {
  records: T[];
  record?: T;
  message?: string;
}

@Injectable({
  providedIn: 'root',
})
export class ApiService {
  private baseUrl = 'http://localhost:8000/api';

  constructor(private http: HttpClient) {}

  // Software Developer APIs
  getAllDevelopers(): Observable<ApiResponse<Developer>> {
    return this.http.get<ApiResponse<Developer>>(
      `${this.baseUrl}/SoftwareDeveloper/readAll.php`
    );
  }

  findDeveloperByName(name: string): Observable<ApiResponse<Developer>> {
    return this.http.get<ApiResponse<Developer>>(
      `${this.baseUrl}/SoftwareDeveloper/findOne.php?name=${encodeURIComponent(
        name
      )}`
    );
  }

  getAllProjects(): Observable<ApiResponse<Project>> {
    return this.http.get<ApiResponse<Project>>(
      `${this.baseUrl}/Project/readAll.php`
    );
  }

  getProjectsByManager(managerId: number): Observable<ApiResponse<Project>> {
    return this.http.get<ApiResponse<Project>>(
      `${this.baseUrl}/Project/readAll.php?projectManagerID=${managerId}`
    );
  }

  assignProject(
    projectName: string,
    projectManagerId: number
  ): Observable<ApiResponse<Project>> {
    return this.http.get<ApiResponse<Project>>(
      `${this.baseUrl}/Project/Assign.php?projectName=${encodeURIComponent(
        projectName
      )}&projectManagerID=${projectManagerId}`
    );
  }
}
