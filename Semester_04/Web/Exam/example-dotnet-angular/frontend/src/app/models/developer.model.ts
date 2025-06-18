export interface Developer {
  id: number;
  name: string;
  skills: string;
}

export interface DevelopersResponse {
  developers: Developer[];
}
