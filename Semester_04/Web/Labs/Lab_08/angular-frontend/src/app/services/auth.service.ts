import { Injectable } from "@angular/core";
import { HttpClient } from "@angular/common/http";
import { Observable, BehaviorSubject } from "rxjs";
import { tap } from "rxjs/operators";

@Injectable({
  providedIn: "root",
})
export class AuthService {
  private apiUrl = "http://localhost:8000/api";
  private isAuthenticatedSubject = new BehaviorSubject<boolean>(
    this.hasToken()
  );

  constructor(private http: HttpClient) {}

  register(username: string, password: string): Observable<any> {
    return this.http.post(`${this.apiUrl}/Auth/register`, {
      username,
      password,
    });
  }

  login(username: string, password: string): Observable<any> {
    return this.http
      .post(`${this.apiUrl}/Auth/login`, { username, password })
      .pipe(
        tap(() => {
          this.isAuthenticatedSubject.next(true);
          localStorage.setItem("authenticated", "true");
        })
      );
  }

  logout(): Observable<any> {
    return this.http.post(`${this.apiUrl}/Auth/logout`, {}).pipe(
      tap(() => {
        this.isAuthenticatedSubject.next(false);
        localStorage.removeItem("authenticated");
      })
    );
  }

  get isAuthenticated(): Observable<boolean> {
    return this.isAuthenticatedSubject.asObservable();
  }

  get isAuthenticatedValue(): boolean {
    return this.isAuthenticatedSubject.value;
  }

  private hasToken(): boolean {
    return localStorage.getItem("authenticated") === "true";
  }
}
