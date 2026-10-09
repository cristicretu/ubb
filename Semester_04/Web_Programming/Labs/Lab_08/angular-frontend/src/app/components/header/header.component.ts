import { Component, OnInit } from "@angular/core";
import { Router, NavigationEnd } from "@angular/router";
import { CategoryService } from "../../services/category.service";
import { Category } from "../../models/category.model";
import { AuthService } from "../../services/auth.service";

@Component({
  selector: "app-header",
  templateUrl: "./header.component.html",
})
export class HeaderComponent implements OnInit {
  isMenuOpen = false;
  previousCategoryTitle: string | null = null;
  previousCategoryId: number | null = null;
  categories: Category[] = [];
  isAuthenticated = false;

  constructor(
    private router: Router,
    private categoryService: CategoryService,
    private authService: AuthService
  ) {}

  ngOnInit(): void {
    this.authService.isAuthenticated.subscribe((auth) => {
      this.isAuthenticated = auth;
    });

    this.categoryService.getCategories().subscribe({
      next: (data) => {
        this.categories = data.records;
        this.loadPreviousCategory();
      },
      error: (err) => {
        this.loadPreviousCategory();
      },
    });

    this.router.events.subscribe((event) => {
      if (event instanceof NavigationEnd) {
        this.loadPreviousCategory();
      }
    });
  }

  logout(): void {
    this.authService.logout().subscribe({
      next: () => {
        this.router.navigate(["/login"]);
      },
    });
  }

  loadPreviousCategory(): void {
    const prev = localStorage.getItem("previousCategory");
    if (prev) {
      const prevObj = JSON.parse(prev);
      this.previousCategoryId = prevObj.id;
      this.previousCategoryTitle =
        prevObj.name ||
        (this.categories.find((c) => c.id === prevObj.id)?.name ?? null);
    }
  }

  goToPreviousCategory(): void {
    if (this.previousCategoryId) {
      this.router.navigate(["/"], {
        queryParams: { category_id: this.previousCategoryId },
      });
    }
  }

  toggleMenu(): void {
    this.isMenuOpen = !this.isMenuOpen;
  }
}
