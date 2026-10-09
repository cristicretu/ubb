import { NgModule } from "@angular/core";
import { RouterModule, Routes } from "@angular/router";
import { CarListComponent } from "./components/car-list/car-list.component";
import { AddCarComponent } from "./components/add-car/add-car.component";
import { EditCarComponent } from "./components/edit-car/edit-car.component";

const routes: Routes = [
  { path: "", component: CarListComponent },
  { path: "add-car", component: AddCarComponent },
  { path: "edit-car/:id", component: EditCarComponent },
  { path: "**", redirectTo: "" },
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule],
})
export class AppRoutingModule {}
