import { Component, inject, OnInit } from '@angular/core';
import { RouterOutlet, Router } from '@angular/router';
import { AuthService } from './services/auth';

@Component({
  selector: 'app-root',
  standalone: true,
  imports: [RouterOutlet],
  templateUrl: './app.html',
  styleUrl: './app.css',
})
export class AppComponent implements OnInit {
  title = 'Project Management';

  private authService = inject(AuthService);
  private router = inject(Router);

  ngOnInit(): void {
    this.checkAuthStatus();
  }

  private checkAuthStatus(): void {
    this.authService.checkAuthStatus().subscribe({
      next: (response) => {
        if (response.isAuthenticated && response.username && response.userId) {
          this.authService.setCurrentUser({
            username: response.username,
            userId: response.userId,
          });
          this.router.navigate(['/dashboard']);
        } else {
          this.router.navigate(['/login']);
        }
      },
      error: (error) => {
        console.error('Auth check failed:', error);
        this.router.navigate(['/login']);
      },
    });
  }
}
