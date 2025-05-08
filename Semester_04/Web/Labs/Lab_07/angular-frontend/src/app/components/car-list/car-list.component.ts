import { Component, OnInit } from "@angular/core";
import { ActivatedRoute, Router } from "@angular/router";
import { Car } from "../../models/car.model";
import { Category } from "../../models/category.model";
import { CarService } from "../../services/car.service";
import { CategoryService } from "../../services/category.service";

@Component({
  selector: "app-car-list",
  templateUrl: "./car-list.component.html",
})
export class CarListComponent implements OnInit {
  cars: Car[] = [];
  categories: Category[] = [];
  selectedCategoryId: number = 0;
  selectedCategory?: Category;
  numberOfCars: number = 0;
  showOnlyBlueCars: boolean = false;

  constructor(
    private carService: CarService,
    private categoryService: CategoryService,
    private route: ActivatedRoute,
    private router: Router
  ) {}

  ngOnInit(): void {
    this.loadCategories();

    this.route.queryParams.subscribe((params) => {
      const categoryId = params["category_id"]
        ? Number(params["category_id"])
        : 0;
      if (categoryId) {
        this.selectedCategoryId = categoryId;
        this.loadCarsFromCategory(this.selectedCategoryId);
      }
    });
  }

  loadCategories(): void {
    this.categoryService.getCategories().subscribe({
      next: (data) => {
        this.categories = data.records;

        if (this.categories.length > 0 && !this.selectedCategoryId) {
          this.selectCategory(this.categories[0]);
        } else if (this.selectedCategoryId) {
          this.selectedCategory = this.categories.find(
            (c) => c.id === this.selectedCategoryId
          );
          this.loadCarsFromCategory(this.selectedCategoryId);
        }
      },
      error: (err) => console.error("Error loading categories:", err),
    });
  }

  toggleShowOnlyBlueCars(): void {
    this.showOnlyBlueCars = !this.showOnlyBlueCars;
    this.loadCarsFromCategory(this.selectedCategoryId);
  }

  selectCategory(category: Category): void {
    if (this.selectedCategory) {
      localStorage.setItem(
        "previousCategory",
        JSON.stringify({
          id: this.selectedCategory.id,
          name: this.selectedCategory.name,
        })
      );
    }
    this.selectedCategoryId = category.id;
    this.selectedCategory = category;
    this.showOnlyBlueCars = false;
    this.loadCarsFromCategory(category.id);

    this.router.navigate([], {
      relativeTo: this.route,
      queryParams: { category_id: category.id },
      queryParamsHandling: "merge",
    });
  }

  loadCarsFromCategory(categoryId: number): void {
    this.carService.getCars(categoryId).subscribe({
      next: (data) => {
        this.cars = data.records.filter(
          (car) => !this.showOnlyBlueCars || car.color === "blue"
        );
        this.numberOfCars = this.cars.length;
      },
      error: (err) => console.error("Error loading cars:", err),
    });
  }

  confirmDeleteCar(car: Car, event: Event): void {
    event.stopPropagation();
    if (confirm("Are you sure you want to delete this car?")) {
      this.carService.deleteCar(car.id).subscribe({
        next: (response) => {
          if (response.success) {
            this.loadCarsFromCategory(this.selectedCategoryId);
          } else {
            alert("Error deleting car: " + response.message);
          }
        },
        error: (err) => {
          console.error("Error deleting car:", err);
          alert("Error deleting car. Please check console for details.");
        },
      });
    }
  }

  formatCurrency(price: number): string {
    return new Intl.NumberFormat("en-US", {
      style: "currency",
      currency: "USD",
    }).format(price);
  }
}
