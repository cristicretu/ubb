import { Component, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import {
  FormBuilder,
  FormGroup,
  Validators,
  ReactiveFormsModule,
} from '@angular/forms';
import { ProductService } from '../../services/product.service';
import { CartItem } from '../../models/product.model';
import { Observable } from 'rxjs';

@Component({
  selector: 'app-products',
  imports: [CommonModule, ReactiveFormsModule],
  templateUrl: './products.html',
  styleUrl: './products.css',
})
export class ProductsComponent implements OnInit {
  productForm: FormGroup;
  searchForm: FormGroup;
  products: { [key: number]: string } = {};
  cart$: Observable<CartItem[]>;
  isLoading = false;
  errorMessage = '';
  successMessage = '';

  constructor(
    private productService: ProductService,
    private formBuilder: FormBuilder
  ) {
    this.cart$ = this.productService.cart$;

    this.productForm = this.formBuilder.group({
      name: ['', [Validators.required]],
      description: ['', [Validators.required]],
    });

    this.searchForm = this.formBuilder.group({
      searchName: ['', [Validators.required]],
    });
  }

  ngOnInit(): void {}

  onAddProduct(): void {
    if (this.productForm.valid) {
      const { name, description } = this.productForm.value;
      this.productService.addProduct(name, description).subscribe({
        next: () => {
          this.successMessage = 'Product added successfully!';
          this.productForm.reset();
        },
        error: () => {
          this.errorMessage = 'Failed to add product';
        },
      });
    }
  }

  onSearchProducts(): void {
    if (this.searchForm.valid) {
      const searchName = this.searchForm.get('searchName')?.value;
      this.isLoading = true;
      this.productService.searchProducts(searchName).subscribe({
        next: (products) => {
          this.products = products;
          this.isLoading = false;
          if (Object.keys(products).length === 0) {
            this.errorMessage = 'No products found';
          } else {
            this.successMessage = `Found ${
              Object.keys(products).length
            } product(s)`;
          }
        },
        error: () => {
          this.errorMessage = 'Failed to search products';
          this.isLoading = false;
        },
      });
    }
  }

  addToCart(productId: number, quantity: number = 1): void {
    this.productService.addToCart(productId, quantity);
    this.successMessage = 'Product added to cart!';
  }

  removeFromCart(productId: number): void {
    this.productService.removeFromCart(productId);
    this.successMessage = 'Product removed from cart!';
  }

  finalizeOrder(): void {
    if (this.productService.cartItems.length === 0) {
      this.errorMessage = 'Cart is empty';
      return;
    }

    this.productService.finalizeOrder().subscribe({
      next: () => {
        this.successMessage = 'Order finalized successfully!';
        this.productService.clearCart();
      },
      error: () => {
        this.errorMessage = 'Failed to finalize order';
      },
    });
  }

  getProductEntries(): Array<[number, string]> {
    return Object.entries(this.products).map(([key, value]) => [
      parseInt(key),
      value,
    ]);
  }

  clearMessages(): void {
    this.errorMessage = '';
    this.successMessage = '';
  }
}
