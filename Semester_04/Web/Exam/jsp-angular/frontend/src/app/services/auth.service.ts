import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { BehaviorSubject, Observable, throwError } from 'rxjs';
import { map, catchError } from 'rxjs/operators';

@Injectable({
  providedIn: 'root',
})
export class AuthService {
  private apiUrl = 'http://localhost:8080';
  private currentUserSubject = new BehaviorSubject<string | null>(
    localStorage.getItem('currentUser')
  );
  public currentUser$ = this.currentUserSubject.asObservable();

  constructor(private http: HttpClient) {}

  login(username: string): Observable<boolean> {
    const body = new URLSearchParams();
    body.append('name', username);

    return this.http
      .post<{
        success: boolean;
        error?: string;
        message?: string;
        user?: string;
      }>(`${this.apiUrl}/login`, body.toString(), {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
        },
      })
      .pipe(
        map((response) => {
          if (response.success && response.user) {
            localStorage.setItem('currentUser', response.user);
            this.currentUserSubject.next(response.user);
            return true;
          }
          return false;
        }),
        catchError((error) => {
          if (error.error && error.error.error) {
            return throwError(() => new Error(error.error.error));
          }
          return throwError(() => new Error('Login failed'));
        })
      );
  }

  logout(): Observable<any> {
    return this.http
      .get<{ success: boolean; message?: string }>(`${this.apiUrl}/logout`)
      .pipe(
        map((response) => {
          localStorage.removeItem('currentUser');
          this.currentUserSubject.next(null);
          return response;
        })
      );
  }

  get currentUserValue(): string | null {
    return this.currentUserSubject.value;
  }

  isLoggedIn(): boolean {
    return !!this.currentUserValue;
  }
}
