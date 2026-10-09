import { Component, OnInit } from "@angular/core";
import { FormBuilder, FormGroup, Validators } from "@angular/forms";
import { Router } from "@angular/router";
import { Car } from "../../models/car.model";
import { Category } from "../../models/category.model";
import { CarService } from "../../services/car.service";
import { CategoryService } from "../../services/category.service";

@Component({
  selector: "app-add-car",
  templateUrl: "./add-car.component.html",
})
export class AddCarComponent implements OnInit {
  carForm: FormGroup;
  categories: Category[] = [];
  alertMessage = "";
  alertType = "";
  isSubmitting = false;

  constructor(
    private fb: FormBuilder,
    private carService: CarService,
    private categoryService: CategoryService,
    private router: Router
  ) {
    this.carForm = this.fb.group({
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
  }

  loadCategories(): void {
    this.categoryService.getCategories().subscribe({
      next: (data) => {
        this.categories = data.records;
        if (this.categories.length > 0) {
          this.carForm.get("category_id")?.setValue(this.categories[0].id);
        }
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

  onSubmit(): void {
    if (this.carForm.invalid) {
      this.markFormGroupTouched(this.carForm);
      return;
    }

    this.isSubmitting = true;
    const car: Car = this.carForm.value;

    this.carService.createCar(car).subscribe({
      next: (response) => {
        this.isSubmitting = false;
        if (response.success) {
          this.router.navigate(["/"], {
            queryParams: { category_id: car.category_id },
          });
        } else {
          this.showAlert(response.message || "Error adding car", "error");
        }
      },
      error: (err) => {
        this.isSubmitting = false;
        console.error("Error adding car:", err);
        this.showAlert("Error adding car. Please try again later.", "error");
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
