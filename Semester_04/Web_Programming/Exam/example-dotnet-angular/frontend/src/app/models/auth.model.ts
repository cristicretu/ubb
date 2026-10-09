export interface User {
  username: string;
  userId: number;
}

export interface LoginRequest {
  username: string;
}

export interface LoginResponse {
  username: string;
  userId: number;
  message?: string;
}

export interface AuthStatusResponse {
  isAuthenticated: boolean;
  username?: string;
  userId?: number;
  debug?: string;
}
