import { Injectable, inject } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { BehaviorSubject, Observable, catchError, of } from 'rxjs';
import {
  AuthStatusResponse,
  LoginRequest,
  LoginResponse,
  User,
} from '../models/auth.model';

@Injectable({
  providedIn: 'root',
})
export class AuthService {
  private http = inject(HttpClient);
  private baseUrl = 'http://localhost:3000';

  private currentUserSubject = new BehaviorSubject<User | null>(null);
  public currentUser$ = this.currentUserSubject.asObservable();

  checkAuthStatus(): Observable<AuthStatusResponse> {
    return this.http
      .get<AuthStatusResponse>(`${this.baseUrl}/api/auth/status`, {
        withCredentials: true,
      })
      .pipe(
        catchError((error) => {
          console.error('Auth check failed:', error);
          return of({ isAuthenticated: false });
        })
      );
  }

  login(request: LoginRequest): Observable<LoginResponse> {
    return this.http.post<LoginResponse>(
      `${this.baseUrl}/api/auth/login`,
      request,
      {
        withCredentials: true,
      }
    );
  }

  logout(): Observable<any> {
    return this.http.post(
      `${this.baseUrl}/api/auth/logout`,
      {},
      {
        withCredentials: true,
      }
    );
  }

  setCurrentUser(user: User | null): void {
    this.currentUserSubject.next(user);
  }

  getCurrentUser(): User | null {
    return this.currentUserSubject.value;
  }

  isAuthenticated(): boolean {
    return this.currentUserSubject.value !== null;
  }
}
