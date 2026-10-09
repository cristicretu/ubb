export interface Product {
  id: number;
  name: string;
  description: string;
}

export interface CartItem {
  productId: number;
  quantity: number;
}
