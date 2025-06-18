import { Injectable } from '@angular/core';
import { HttpClient, HttpParams } from '@angular/common/http';
import { Observable } from 'rxjs';
import { Project } from '../models/project.model';

@Injectable({
  providedIn: 'root',
})
export class ProjectService {
  private apiUrl = 'http://localhost:8080/project';

  constructor(private http: HttpClient) {}

  getAllProjects(): Observable<Project[]> {
    const params = new HttpParams().set('action', 'readAll');
    return this.http.get<Project[]>(this.apiUrl, { params });
  }

  getProjectsByManagerId(projectManagerId: number): Observable<Project[]> {
    const params = new HttpParams()
      .set('action', 'readAllByProjectManagerID')
      .set('projectManagerId', projectManagerId.toString());
    return this.http.get<Project[]>(this.apiUrl, { params });
  }

  createProject(project: Project): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('action', 'create');
    formData.append('name', project.name);
    formData.append('projectManagerId', project.projectManagerId.toString());
    formData.append('description', project.description);
    formData.append('members', project.members);

    return this.http.post<{ success: boolean }>(this.apiUrl, formData);
  }

  updateProject(project: Project): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('id', project.id!.toString());
    formData.append('projectManagerId', project.projectManagerId.toString());
    formData.append('name', project.name);
    formData.append('description', project.description);
    formData.append('members', project.members);

    return this.http.put<{ success: boolean }>(this.apiUrl, formData);
  }

  deleteProject(id: number): Observable<{ success: boolean }> {
    const formData = new FormData();
    formData.append('id', id.toString());

    return this.http.delete<{ success: boolean }>(this.apiUrl, {
      body: formData,
    });
  }

  assignProject(
    projectName: string,
    projectManagerId: number
  ): Observable<Project> {
    const formData = new FormData();
    formData.append('action', 'assignProject');
    formData.append('projectName', projectName);
    formData.append('projectManagerId', projectManagerId.toString());

    return this.http.post<Project>(this.apiUrl, formData);
  }
}
