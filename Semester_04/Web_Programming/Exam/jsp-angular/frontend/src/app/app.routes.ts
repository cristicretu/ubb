import { Routes } from '@angular/router';
import { LoginComponent } from './components/login/login';
import { DashboardComponent } from './components/dashboard/dashboard';
import { ProjectsComponent } from './components/projects/projects';
import { DevelopersComponent } from './components/developers/developers';
import { ProductsComponent } from './components/products/products';

export const routes: Routes = [
  { path: '', redirectTo: '/login', pathMatch: 'full' },
  { path: 'login', component: LoginComponent },
  {
    path: 'dashboard',
    component: DashboardComponent,
    children: [
      { path: '', redirectTo: 'projects', pathMatch: 'full' },
      { path: 'projects', component: ProjectsComponent },
      { path: 'developers', component: DevelopersComponent },
      { path: 'products', component: ProductsComponent },
    ],
  },
  { path: '**', redirectTo: '/login' },
];
