import { Injectable, inject } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';
import {
  ProjectsResponse,
  CreateProjectRequest,
  CreateProjectResponse,
} from '../models/project.model';

@Injectable({
  providedIn: 'root',
})
export class ProjectService {
  private http = inject(HttpClient);
  private baseUrl = 'http://localhost:3000';

  getProjects(): Observable<ProjectsResponse> {
    return this.http.get<ProjectsResponse>(`${this.baseUrl}/api/projects`, {
      withCredentials: true,
    });
  }

  createProject(
    request: CreateProjectRequest
  ): Observable<CreateProjectResponse> {
    return this.http.post<CreateProjectResponse>(
      `${this.baseUrl}/api/projects`,
      request,
      {
        withCredentials: true,
      }
    );
  }
}
