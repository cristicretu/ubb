import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { ApiService, Developer, Project } from '../services/api.service';

@Component({
  selector: 'app-dashboard',
  standalone: true,
  imports: [CommonModule, FormsModule],
  template: `
    <div class="min-h-screen bg-gray-100 p-6">
      <div class="max-w-7xl mx-auto">
        <h1 class="text-3xl font-bold text-gray-800 mb-6">
          Project Management
        </h1>

        <div class="bg-white p-6 rounded-lg shadow-md">
          <!-- All Projects Section -->
          <h2 class="text-xl font-bold text-gray-800 mb-4">All Projects</h2>
          <div class="mb-6">
            <div
              *ngFor="let project of allProjects"
              class="flex flex-row gap-4 items-center justify-between p-4 border border-gray-300 rounded-md mb-2"
            >
              <h3 class="font-medium">{{ project.name }}</h3>
              <p class="text-gray-600">
                {{ project.description || 'No description' }}
              </p>
              <p class="text-sm text-gray-500">
                Members: {{ project.members || 'None' }}
              </p>
            </div>
          </div>

          <!-- Your Managed Projects Section -->
          <h2 class="text-xl font-bold text-gray-800 mb-4">
            Your Managed Projects
          </h2>
          <div class="mb-6">
            <div
              *ngFor="let project of yourProjects"
              class="flex flex-row gap-4 items-center justify-between p-4 border border-gray-300 rounded-md mb-2"
            >
              <h3 class="font-medium">{{ project.name }}</h3>
              <p class="text-gray-600">
                {{ project.description || 'No description' }}
              </p>
              <p class="text-sm text-gray-500">
                Members: {{ project.members || 'None' }}
              </p>
            </div>
            <div
              *ngIf="yourProjects.length === 0"
              class="p-4 text-gray-500 italic"
            >
              You are not managing any projects
            </div>
          </div>

          <!-- Projects You're Member Of Section -->
          <h2 class="text-xl font-bold text-gray-800 mb-4">
            Projects You're Member Of
          </h2>
          <div class="mb-6">
            <div
              *ngFor="let project of memberProjects"
              class="flex flex-row gap-4 items-center justify-between p-4 border border-gray-300 rounded-md mb-2"
            >
              <h3 class="font-medium">{{ project.name }}</h3>
              <p class="text-gray-600">
                {{ project.description || 'No description' }}
              </p>
              <p class="text-sm text-gray-500">
                Members: {{ project.members }}
              </p>
            </div>
            <div
              *ngIf="memberProjects.length === 0"
              class="p-4 text-gray-500 italic"
            >
              You are not a member of any projects
            </div>
          </div>

          <!-- Assign Project Section -->
          <h2 class="text-xl font-bold text-gray-800 mb-4">Assign Project</h2>
          <form (ngSubmit)="onAssignProject()" class="mb-6">
            <div class="flex flex-col space-y-4">
              <input
                type="text"
                [(ngModel)]="newProject.name"
                name="projectName"
                placeholder="Project Name"
                class="p-3 border border-gray-300 rounded-md"
                required
              />
              <input
                type="text"
                [(ngModel)]="newProject.managerName"
                name="managerName"
                placeholder="Project Manager Name"
                class="p-3 border border-gray-300 rounded-md"
                required
              />
              <button
                type="submit"
                class="bg-blue-500 text-white px-6 py-3 rounded-md hover:bg-blue-600"
                [disabled]="assigningProject"
              >
                {{ assigningProject ? 'Adding...' : 'Add Project' }}
              </button>
            </div>
          </form>

          <!-- Developers Section -->
          <h2 class="text-xl font-bold text-gray-800 mb-4">Developers</h2>
          <div class="flex flex-row gap-4 mb-4">
            <button
              (click)="showAllDevelopers()"
              class="bg-blue-500 text-white px-4 py-2 rounded-md hover:bg-blue-600"
            >
              Show All Developers
            </button>
            <input
              type="text"
              [(ngModel)]="skillFilter"
              name="skillFilter"
              placeholder="Filter by skill (e.g., Java)"
              class="p-2 border border-gray-300 rounded-md flex-1"
            />
            <button
              (click)="filterDevelopers()"
              class="bg-green-500 text-white px-4 py-2 rounded-md hover:bg-green-600"
            >
              Filter
            </button>
          </div>

          <div class="space-y-2">
            <div
              *ngFor="let dev of filteredDevelopers"
              class="p-3 border border-gray-300 rounded-md"
            >
              {{ dev.name }} - {{ dev.skills }}
            </div>
            <div
              *ngIf="filteredDevelopers.length === 0 && developersLoaded"
              class="p-3 text-gray-500 italic"
            >
              No developers found with this skill
            </div>
          </div>
        </div>
      </div>
    </div>
  `,
})
export class DashboardComponent implements OnInit {
  allProjects: Project[] = [];
  yourProjects: Project[] = [];
  memberProjects: Project[] = [];
  allDevelopers: Developer[] = [];
  filteredDevelopers: Developer[] = [];

  newProject = { name: '', managerName: '' };
  skillFilter = '';
  assigningProject = false;
  developersLoaded = false;

  currentUser = '';
  currentUserId: number | null = null;

  constructor(private apiService: ApiService) {}

  ngOnInit() {
    this.currentUser = localStorage.getItem('currentUser') || '';
    this.loadData();
  }

  async loadData() {
    try {
      // Load all projects
      this.apiService.getAllProjects().subscribe({
        next: (response) => {
          this.allProjects = response.records || [];
          this.filterMemberProjects();
        },
        error: (error) => console.error('Error loading all projects:', error),
      });

      // Get current user ID and load their managed projects
      if (this.currentUser) {
        this.apiService.findDeveloperByName(this.currentUser).subscribe({
          next: (response) => {
            if (response.record) {
              this.currentUserId = response.record.id;
              this.loadManagedProjects();
            }
          },
          error: (error) => console.error('Error finding user:', error),
        });
      }
    } catch (error) {
      console.error('Error loading data:', error);
    }
  }

  loadManagedProjects() {
    if (this.currentUserId) {
      this.apiService.getProjectsByManager(this.currentUserId).subscribe({
        next: (response) => {
          this.yourProjects = response.records || [];
        },
        error: (error) =>
          console.error('Error loading managed projects:', error),
      });
    }
  }

  filterMemberProjects() {
    this.memberProjects = this.allProjects.filter(
      (project) => project.members && project.members.includes(this.currentUser)
    );
  }

  showAllDevelopers() {
    this.apiService.getAllDevelopers().subscribe({
      next: (response) => {
        this.allDevelopers = response.records || [];
        this.filteredDevelopers = this.allDevelopers;
        this.developersLoaded = true;
      },
      error: (error) => console.error('Error loading developers:', error),
    });
  }

  filterDevelopers() {
    if (this.skillFilter.trim()) {
      this.filteredDevelopers = this.allDevelopers.filter((dev) =>
        dev.skills.toLowerCase().includes(this.skillFilter.toLowerCase())
      );
    } else {
      this.filteredDevelopers = this.allDevelopers;
    }
  }

  async onAssignProject() {
    if (!this.newProject.name || !this.newProject.managerName) {
      alert('Please fill in all fields');
      return;
    }

    this.assigningProject = true;

    try {
      // First, check if the user exists
      this.apiService
        .findDeveloperByName(this.newProject.managerName)
        .subscribe({
          next: (response) => {
            if (response.record) {
              // User exists, assign the project
              this.apiService
                .assignProject(this.newProject.name, response.record!.id)
                .subscribe({
                  next: (assignResponse) => {
                    if (assignResponse.record) {
                      alert('Project assigned successfully!');
                      this.newProject = { name: '', managerName: '' };
                      this.loadData(); // Refresh data
                    } else {
                      alert(
                        'Project assignment failed: ' +
                          (assignResponse.message || 'Unknown error')
                      );
                    }
                    this.assigningProject = false;
                  },
                  error: (error) => {
                    console.error('Assignment error:', error);
                    alert('Error assigning project');
                    this.assigningProject = false;
                  },
                });
            } else {
              alert('Developer not found!');
              this.assigningProject = false;
            }
          },
          error: (error) => {
            console.error('Error finding developer:', error);
            alert('Error finding developer');
            this.assigningProject = false;
          },
        });
    } catch (error) {
      console.error('Error:', error);
      alert('An error occurred');
      this.assigningProject = false;
    }
  }
}
