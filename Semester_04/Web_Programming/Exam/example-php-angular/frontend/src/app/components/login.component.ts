import { Component } from '@angular/core';
import { Router } from '@angular/router';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';

@Component({
  selector: 'app-login',
  standalone: true,
  imports: [FormsModule, CommonModule],
  template: `
    <div class="min-h-screen bg-gray-100 flex items-center justify-center">
      <div class="bg-white p-8 rounded-lg shadow-md w-full max-w-md">
        <h1 class="text-3xl font-bold text-gray-800 mb-6 text-center">Login</h1>
        <form (ngSubmit)="onLogin()" #loginForm="ngForm">
          <div class="mb-4">
            <input
              type="text"
              [(ngModel)]="username"
              name="username"
              placeholder="Username"
              required
              class="w-full p-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <button
            type="submit"
            class="w-full bg-blue-500 text-white py-3 px-4 rounded-md hover:bg-blue-600 transition-colors"
            [disabled]="!loginForm.form.valid"
          >
            Login
          </button>
        </form>
      </div>
    </div>
  `,
})
export class LoginComponent {
  username = '';

  constructor(private router: Router) {}

  onLogin() {
    if (this.username.trim() === '') {
      alert('Please enter a username');
      return;
    }

    // Save to localStorage like the original PHP app
    localStorage.setItem('currentUser', this.username);

    // Navigate to dashboard
    this.router.navigate(['/dashboard']);
  }
}
