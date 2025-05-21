import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable, BehaviorSubject } from "rxjs";
import { tap } from "rxjs/operators";

@Injectable({
  providedIn: "root",
})
export class AuthService {
  private apiUrl = "http://localhost:8000/api";
  private isAuthenticatedSubject = new BehaviorSubject<boolean>(false);

  constructor(private http: HttpClient) {
    this.checkAuthStatus();
  }

  checkAuthStatus(): void {
    this.http
      .get<{ isAuthenticated: boolean }>(`${this.apiUrl}/Auth/status`, {
        withCredentials: true,
      })
      .subscribe({
        next: (response) => {
          console.log("Auth status check:", response);
          this.isAuthenticatedSubject.next(response.isAuthenticated);
        },
        error: () => {
          console.log("Not authenticated");
          this.isAuthenticatedSubject.next(false);
        },
      });
  }

  register(username: string, password: string): Observable<any> {
    return this.http.post(
      `${this.apiUrl}/Auth/register`,
      {
        username,
        password,
      },
      { withCredentials: true }
    );
  }

  login(username: string, password: string): Observable<any> {
    return this.http
      .post(
        `${this.apiUrl}/Auth/login`,
        { username, password },
        {
          withCredentials: true,
        }
      )
      .pipe(
        tap((response) => {
          console.log("Login response:", response);
          this.isAuthenticatedSubject.next(true);
        })
      );
  }

  logout(): Observable<any> {
    return this.http
      .post(
        `${this.apiUrl}/Auth/logout`,
        {},
        {
          withCredentials: true,
        }
      )
      .pipe(
        tap(() => {
          this.isAuthenticatedSubject.next(false);
        })
      );
  }

  get isAuthenticated(): Observable<boolean> {
    return this.isAuthenticatedSubject.asObservable();
  }

  get isAuthenticatedValue(): boolean {
    return this.isAuthenticatedSubject.value;
  }
}
