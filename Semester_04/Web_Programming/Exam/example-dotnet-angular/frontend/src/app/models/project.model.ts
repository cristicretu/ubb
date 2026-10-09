export interface Project {
  id: number;
  name: string;
  managerId: number;
  managerName: string;
}

export interface ProjectsResponse {
  allProjects: Project[];
  yourProjects: Project[];
  memberProjects: Project[];
}

export interface CreateProjectRequest {
  projectName: string;
  projectManagerName: string;
}

export interface CreateProjectResponse {
  message: string;
  projectId?: number;
}
