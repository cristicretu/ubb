import { io, Socket } from 'socket.io-client';
import { Car } from '../types/Car';

const SOCKET_URL = 'http://localhost:3000';

export type CarWithUpdatedAt = Car & { updatedAt: string };
type CarEventHandler = (car: CarWithUpdatedAt) => void;
type DeleteEventHandler = (id: string) => void;

export class WebSocketService {
  private socket: Socket | null = null;
  private isConnected: boolean = false;
  private shouldSuppressErrors: boolean = false;
  private listeners: {
    created: CarEventHandler[];
    updated: CarEventHandler[];
    deleted: DeleteEventHandler[];
  } = {
      created: [],
      updated: [],
      deleted: [],
    };

  connect() {
    if (this.socket && this.isConnected) {
      console.log('[WebSocket] Already connected');
      return;
    }

    console.log('[WebSocket] Connecting to server...');
    this.shouldSuppressErrors = false;

    this.socket = io(SOCKET_URL, {
      transports: ['websocket'],
      reconnection: true,
      reconnectionDelay: 2000,
      reconnectionDelayMax: 5000,
      reconnectionAttempts: 3,
      timeout: 5000,
    });

    this.socket.on('connect', () => {
      this.isConnected = true;
      console.log('[WebSocket] Connected:', this.socket?.id);
    });

    this.socket.on('disconnect', () => {
      this.isConnected = false;
      console.log('[WebSocket] Disconnected');
    });

    this.socket.on('connect_error', () => {
      this.isConnected = false;
      if (!this.shouldSuppressErrors) {
        this.shouldSuppressErrors = true;
      }
    });

    this.socket.on('reconnect_failed', () => {
      this.isConnected = false;
      if (!this.shouldSuppressErrors) {
        console.log('[WebSocket] Reconnection failed - server unreachable');
        this.shouldSuppressErrors = true;
      }
    });

    this.socket.on('car:created', (data) => {
      console.log('[WebSocket] car:created received:', data.id);
      this.listeners.created.forEach(handler => handler(this.convertToCar(data)));
    });

    this.socket.on('car:updated', (data) => {
      console.log('[WebSocket] car:updated received:', data.id);
      this.listeners.updated.forEach(handler => handler(this.convertToCar(data)));
    });

    this.socket.on('car:deleted', (data) => {
      console.log('[WebSocket] car:deleted received:', data.id);
      this.listeners.deleted.forEach(handler => handler(data.id));
    });
  }

  disconnect() {
    if (this.socket) {
      this.socket.disconnect();
      this.socket = null;
      this.isConnected = false;
      this.shouldSuppressErrors = false;
      console.log('[WebSocket] Manually disconnected');
    }
  }

  onCarCreated(handler: CarEventHandler) {
    this.listeners.created.push(handler);
  }

  onCarUpdated(handler: CarEventHandler) {
    this.listeners.updated.push(handler);
  }

  onCarDeleted(handler: DeleteEventHandler) {
    this.listeners.deleted.push(handler);
  }

  clearListeners() {
    this.listeners = {
      created: [],
      updated: [],
      deleted: [],
    };
  }

  getIsConnected(): boolean {
    return this.isConnected;
  }

  private convertToCar(data: any): CarWithUpdatedAt {
    return {
      id: data.id,
      make: data.make,
      model: data.model,
      year: data.year,
      licensePlate: data.licensePlate,
      currentMileage: data.currentMileage,
      lastITPDate: new Date(data.lastITPDate),
      nextITPDate: new Date(data.nextITPDate),
      rovinietaExpiryDate: new Date(data.rovinietaExpiryDate),
      carDescription: data.carDescription,
      updatedAt: data.updatedAt || new Date().toISOString(),
    };
  }
}

export const webSocketService = new WebSocketService();
