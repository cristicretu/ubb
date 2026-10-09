import { Component, OnInit } from "@angular/core";
import { FormBuilder, FormGroup, Validators } from "@angular/forms";
import { ActivatedRoute, Router } from "@angular/router";
import { Car } from "../../models/car.model";
import { Category } from "../../models/category.model";
import { CarService } from "../../services/car.service";
import { CategoryService } from "../../services/category.service";

@Component({
  selector: "app-edit-car",
  templateUrl: "./edit-car.component.html",
})
export class EditCarComponent implements OnInit {
  carForm: FormGroup;
  carId: number = 0;
  categories: Category[] = [];
  alertMessage = "";
  alertType = "";
  isLoading = true;
  isSubmitting = false;

  constructor(
    private fb: FormBuilder,
    private carService: CarService,
    private categoryService: CategoryService,
    private route: ActivatedRoute,
    private router: Router
  ) {
    this.carForm = this.fb.group({
      id: [""],
      model: ["", [Validators.required, Validators.minLength(2)]],
      engine_power: ["", Validators.required],
      fuel_type: ["", Validators.required],
      color: ["", Validators.required],
      year: [
        "",
        [
          Validators.required,
          Validators.min(1900),
          Validators.max(new Date().getFullYear() + 1),
        ],
      ],
      price: ["", [Validators.required, Validators.min(0)]],
      features: [""],
      category_id: ["", Validators.required],
    });
  }

  ngOnInit(): void {
    this.loadCategories();

    this.route.params.subscribe((params) => {
      this.carId = +params["id"];
      this.loadCar(this.carId);
    });
  }

  loadCategories(): void {
    this.categoryService.getCategories().subscribe({
      next: (data) => {
        this.categories = data.records;
      },
      error: (err) => {
        console.error("Error loading categories:", err);
        this.showAlert(
          "Error loading categories. Please try again later.",
          "error"
        );
      },
    });
  }

  loadCar(id: number): void {
    this.isLoading = true;
    this.carService.getCar(id).subscribe({
      next: (response) => {
        this.isLoading = false;
        if (response.data) {
          this.carForm.patchValue(response.data);
        } else {
          this.showAlert("Car not found", "error");
          setTimeout(() => {
            this.router.navigate(["/"]);
          }, 2000);
        }
      },
      error: (err) => {
        this.isLoading = false;
        console.error("Error loading car:", err);
        this.showAlert("Error loading car. Please try again later.", "error");
      },
    });
  }

  onSubmit(): void {
    if (this.carForm.invalid) {
      this.markFormGroupTouched(this.carForm);
      return;
    }

    this.isSubmitting = true;
    const car: Car = this.carForm.value;

    this.carService.updateCar(car).subscribe({
      next: (response) => {
        this.isSubmitting = false;
        if (response.success) {
          this.router.navigate(["/"], {
            queryParams: { category_id: car.category_id },
          });
        } else {
          this.showAlert(response.message || "Error updating car", "error");
        }
      },
      error: (err) => {
        this.isSubmitting = false;
        console.error("Error updating car:", err);
        this.showAlert("Error updating car. Please try again later.", "error");
      },
    });
  }

  markFormGroupTouched(formGroup: FormGroup): void {
    Object.values(formGroup.controls).forEach((control) => {
      control.markAsTouched();
      if ((control as FormGroup).controls) {
        this.markFormGroupTouched(control as FormGroup);
      }
    });
  }

  showAlert(message: string, type: "success" | "error"): void {
    this.alertMessage = message;
    this.alertType = type;
    setTimeout(() => {
      this.alertMessage = "";
    }, 5000);
  }
}
