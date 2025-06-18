import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import {
  FormBuilder,
  FormGroup,
  Validators,
  ReactiveFormsModule,
} from '@angular/forms';
import { SoftwareDeveloper } from '../../models/software-developer.model';
import { SoftwareDeveloperService } from '../../services/software-developer.service';

@Component({
  selector: 'app-developers',
  imports: [CommonModule, ReactiveFormsModule],
  templateUrl: './developers.html',
  styleUrl: './developers.css',
})
export class DevelopersComponent implements OnInit {
  developers: SoftwareDeveloper[] = [];
  developerForm: FormGroup;
  editingDeveloper: SoftwareDeveloper | null = null;
  isLoading = false;
  errorMessage = '';
  successMessage = '';

  constructor(
    private developerService: SoftwareDeveloperService,
    private formBuilder: FormBuilder
  ) {
    this.developerForm = this.formBuilder.group({
      name: ['', [Validators.required]],
      age: ['', [Validators.required, Validators.min(18), Validators.max(100)]],
      skills: ['', [Validators.required]],
    });
  }

  ngOnInit(): void {
    this.loadDevelopers();
  }

  loadDevelopers(): void {
    this.isLoading = true;
    this.developerService.getAllDevelopers().subscribe({
      next: (developers) => {
        this.developers = developers;
        this.isLoading = false;
      },
      error: (error) => {
        this.errorMessage = 'Failed to load developers';
        this.isLoading = false;
      },
    });
  }

  onSubmit(): void {
    if (this.developerForm.valid) {
      const formValue = this.developerForm.value;
      const developer: SoftwareDeveloper = {
        name: formValue.name,
        age: parseInt(formValue.age),
        skills: formValue.skills,
      };

      if (this.editingDeveloper) {
        developer.id = this.editingDeveloper.id;
        this.updateDeveloper(developer);
      } else {
        this.createDeveloper(developer);
      }
    }
  }

  createDeveloper(developer: SoftwareDeveloper): void {
    this.developerService.createDeveloper(developer).subscribe({
      next: (response) => {
        if (response.success) {
          this.successMessage = 'Developer created successfully';
          this.resetForm();
          this.loadDevelopers();
        } else {
          this.errorMessage = 'Failed to create developer';
        }
      },
      error: (error) => {
        this.errorMessage = 'Error creating developer';
      },
    });
  }

  updateDeveloper(developer: SoftwareDeveloper): void {
    this.developerService.updateDeveloper(developer).subscribe({
      next: (response) => {
        if (response.success) {
          this.successMessage = 'Developer updated successfully';
          this.resetForm();
          this.loadDevelopers();
        } else {
          this.errorMessage = 'Failed to update developer';
        }
      },
      error: (error) => {
        this.errorMessage = 'Error updating developer';
      },
    });
  }

  editDeveloper(developer: SoftwareDeveloper): void {
    this.editingDeveloper = developer;
    this.developerForm.patchValue({
      name: developer.name,
      age: developer.age,
      skills: developer.skills,
    });
  }

  deleteDeveloper(id: number): void {
    if (confirm('Are you sure you want to delete this developer?')) {
      this.developerService.deleteDeveloper(id).subscribe({
        next: (response) => {
          if (response.success) {
            this.successMessage = 'Developer deleted successfully';
            this.loadDevelopers();
          } else {
            this.errorMessage = 'Failed to delete developer';
          }
        },
        error: (error) => {
          this.errorMessage = 'Error deleting developer';
        },
      });
    }
  }

  resetForm(): void {
    this.developerForm.reset();
    this.editingDeveloper = null;
    this.clearMessages();
  }

  clearMessages(): void {
    this.errorMessage = '';
    this.successMessage = '';
  }
}
