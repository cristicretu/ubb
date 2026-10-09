import { NgModule } from "@angular/core";
import { RouterModule, Routes } from "@angular/router";
import { CarListComponent } from "./components/car-list/car-list.component";
import { AddCarComponent } from "./components/add-car/add-car.component";
import { EditCarComponent } from "./components/edit-car/edit-car.component";
import { LoginComponent } from "./components/login/login.component";
import { RegisterComponent } from "./components/register/register.component";
import { AuthGuard } from "./guards/auth.guard";

const routes: Routes = [
  { path: "login", component: LoginComponent },
  { path: "register", component: RegisterComponent },
  { path: "", component: CarListComponent, canActivate: [AuthGuard] },
  { path: "add-car", component: AddCarComponent, canActivate: [AuthGuard] },
  {
    path: "edit-car/:id",
    component: EditCarComponent,
    canActivate: [AuthGuard],
  },
  { path: "**", redirectTo: "" },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule],
})
export class AppRoutingModule {}
