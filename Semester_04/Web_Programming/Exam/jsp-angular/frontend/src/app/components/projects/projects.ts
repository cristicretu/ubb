import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import {
  FormBuilder,
  FormGroup,
  Validators,
  ReactiveFormsModule,
} from '@angular/forms';
import { Project } from '../../models/project.model';
import { ProjectService } from '../../services/project.service';

@Component({
  selector: 'app-projects',
  imports: [CommonModule, ReactiveFormsModule],
  templateUrl: './projects.html',
  styleUrl: './projects.css',
})
export class ProjectsComponent implements OnInit {
  projects: Project[] = [];
  projectForm: FormGroup;
  editingProject: Project | null = null;
  isLoading = false;
  errorMessage = '';
  successMessage = '';

  constructor(
    private projectService: ProjectService,
    private formBuilder: FormBuilder
  ) {
    this.projectForm = this.formBuilder.group({
      name: ['', [Validators.required]],
      projectManagerId: ['', [Validators.required, Validators.min(1)]],
      description: ['', [Validators.required]],
      members: ['', [Validators.required]],
    });
  }

  ngOnInit(): void {
    this.loadProjects();
  }

  loadProjects(): void {
    this.isLoading = true;
    this.projectService.getAllProjects().subscribe({
      next: (projects) => {
        this.projects = projects;
        this.isLoading = false;
      },
      error: (error) => {
        this.errorMessage = 'Failed to load projects';
        this.isLoading = false;
      },
    });
  }

  onSubmit(): void {
    if (this.projectForm.valid) {
      const formValue = this.projectForm.value;
      const project: Project = {
        name: formValue.name,
        projectManagerId: parseInt(formValue.projectManagerId),
        description: formValue.description,
        members: formValue.members,
      };

      if (this.editingProject) {
        project.id = this.editingProject.id;
        this.updateProject(project);
      } else {
        this.createProject(project);
      }
    }
  }

  createProject(project: Project): void {
    this.projectService.createProject(project).subscribe({
      next: (response) => {
        if (response.success) {
          this.successMessage = 'Project created successfully';
          this.resetForm();
          this.loadProjects();
        } else {
          this.errorMessage = 'Failed to create project';
        }
      },
      error: (error) => {
        this.errorMessage = 'Error creating project';
      },
    });
  }

  updateProject(project: Project): void {
    this.projectService.updateProject(project).subscribe({
      next: (response) => {
        if (response.success) {
          this.successMessage = 'Project updated successfully';
          this.resetForm();
          this.loadProjects();
        } else {
          this.errorMessage = 'Failed to update project';
        }
      },
      error: (error) => {
        this.errorMessage = 'Error updating project';
      },
    });
  }

  editProject(project: Project): void {
    this.editingProject = project;
    this.projectForm.patchValue({
      name: project.name,
      projectManagerId: project.projectManagerId,
      description: project.description,
      members: project.members,
    });
  }

  deleteProject(id: number): void {
    if (confirm('Are you sure you want to delete this project?')) {
      this.projectService.deleteProject(id).subscribe({
        next: (response) => {
          if (response.success) {
            this.successMessage = 'Project deleted successfully';
            this.loadProjects();
          } else {
            this.errorMessage = 'Failed to delete project';
          }
        },
        error: (error) => {
          this.errorMessage = 'Error deleting project';
        },
      });
    }
  }

  resetForm(): void {
    this.projectForm.reset();
    this.editingProject = null;
    this.clearMessages();
  }

  clearMessages(): void {
    this.errorMessage = '';
    this.successMessage = '';
  }
}
