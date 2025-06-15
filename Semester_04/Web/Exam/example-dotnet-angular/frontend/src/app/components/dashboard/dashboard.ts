import { Component, inject, OnInit } from '@angular/core';
import { Router } from '@angular/router';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { AuthService } from '../../services/auth';
import { ProjectService } from '../../services/project';
import { DeveloperService } from '../../services/developer';
import { User } from '../../models/auth.model';
import {
  ProjectsResponse,
  CreateProjectRequest,
} from '../../models/project.model';
import { Developer } from '../../models/developer.model';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [CommonModule, FormsModule],
  templateUrl: './dashboard.html',
  styleUrl: './dashboard.css',
})
export class DashboardComponent implements OnInit {
  private authService = inject(AuthService);
  private projectService = inject(ProjectService);
  private developerService = inject(DeveloperService);
  private router = inject(Router);

  currentUser: User | null = null;
  projects: ProjectsResponse | null = null;
  developers: Developer[] = [];

  loadingProjects = false;
  loadingDevelopers = false;
  assigningProject = false;
  showDevelopers = false;

  successMessage = '';
  errorMessage = '';
  skillFilter = '';

  newProject: CreateProjectRequest = {
    projectName: '',
    projectManagerName: '',
  };

  ngOnInit(): void {
    this.currentUser = this.authService.getCurrentUser();

    if (!this.currentUser) {
      this.router.navigate(['/login']);
      return;
    }

    this.loadProjects();
  }

  loadProjects(): void {
    this.loadingProjects = true;
    this.hideMessages();

    this.projectService.getProjects().subscribe({
      next: (response) => {
        this.projects = response;
        this.loadingProjects = false;
      },
      error: (error) => {
        console.error('Load projects error:', error);
        if (error.status === 401) {
          this.router.navigate(['/login']);
        } else {
          this.errorMessage = 'Failed to load projects';
        }
        this.loadingProjects = false;
      },
    });
  }

  assignProject(): void {
    if (
      !this.newProject.projectName.trim() ||
      !this.newProject.projectManagerName.trim()
    ) {
      this.errorMessage = 'Please fill in all fields';
      return;
    }

    this.assigningProject = true;
    this.hideMessages();

    this.projectService.createProject(this.newProject).subscribe({
      next: (response) => {
        this.successMessage = response.message;
        this.newProject = { projectName: '', projectManagerName: '' };
        this.loadProjects(); // Refresh projects
        this.assigningProject = false;
      },
      error: (error) => {
        console.error('Assign project error:', error);
        this.errorMessage = error.error?.message || 'Failed to assign project';
        this.assigningProject = false;
      },
    });
  }

  showAllDevelopers(): void {
    this.skillFilter = '';
    this.loadDevelopers();
    this.showDevelopers = true;
  }

  filterDevelopers(): void {
    this.loadDevelopers(this.skillFilter.trim() || undefined);
    this.showDevelopers = true;
  }

  private loadDevelopers(skill?: string): void {
    this.loadingDevelopers = true;
    this.hideMessages();

    this.developerService.getDevelopers(skill).subscribe({
      next: (response) => {
        this.developers = response.developers;
        this.loadingDevelopers = false;
      },
      error: (error) => {
        console.error('Load developers error:', error);
        this.errorMessage = 'Failed to load developers';
        this.loadingDevelopers = false;
      },
    });
  }

  logout(): void {
    this.authService.logout().subscribe({
      next: () => {
        this.authService.setCurrentUser(null);
        this.router.navigate(['/login']);
      },
      error: (error) => {
        console.error('Logout error:', error);
        // Even if logout fails on server, clear local state
        this.authService.setCurrentUser(null);
        this.router.navigate(['/login']);
      },
    });
  }

  private hideMessages(): void {
    this.successMessage = '';
    this.errorMessage = '';
  }
}
