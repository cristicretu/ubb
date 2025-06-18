import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable, BehaviorSubject } from 'rxjs';
import { Product, CartItem } from '../models/product.model';

@Injectable({
  providedIn: 'root',
})
export class ProductService {
  private apiUrl = 'http://localhost:8080/main';
  private cartSubject = new BehaviorSubject<CartItem[]>([]);
  public cart$ = this.cartSubject.asObservable();

  constructor(private http: HttpClient) {
    // Load cart from localStorage on service initialization
    const savedCart = localStorage.getItem('cart');
    if (savedCart) {
      this.cartSubject.next(JSON.parse(savedCart));
    }
  }

  addProduct(name: string, description: string): Observable<any> {
    const formData = new FormData();
    formData.append('action', 'add_product');
    formData.append('product_name', name);
    formData.append('product_description', description);

    return this.http.post(this.apiUrl, formData, { responseType: 'text' });
  }

  searchProducts(name: string): Observable<{ [key: number]: string }> {
    const formData = new FormData();
    formData.append('action', 'search_product');
    formData.append('name', name);

    return this.http.post<{ [key: number]: string }>(this.apiUrl, formData);
  }

  addToCart(productId: number, quantity: number = 1): void {
    const currentCart = this.cartSubject.value;
    const existingItem = currentCart.find(
      (item) => item.productId === productId
    );

    if (existingItem) {
      existingItem.quantity += quantity;
    } else {
      currentCart.push({ productId, quantity });
    }

    this.cartSubject.next([...currentCart]);
    localStorage.setItem('cart', JSON.stringify(currentCart));
  }

  removeFromCart(productId: number): void {
    const currentCart = this.cartSubject.value.filter(
      (item) => item.productId !== productId
    );
    this.cartSubject.next(currentCart);
    localStorage.setItem('cart', JSON.stringify(currentCart));
  }

  clearCart(): void {
    this.cartSubject.next([]);
    localStorage.removeItem('cart');
  }

  finalizeOrder(): Observable<any> {
    const formData = new FormData();
    formData.append('action', 'finalize_order');

    return this.http.post(this.apiUrl, formData, { responseType: 'text' });
  }

  get cartItems(): CartItem[] {
    return this.cartSubject.value;
  }
}
