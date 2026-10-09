import NetInfo from '@react-native-community/netinfo';

export class NetworkService {
  private isOnline: boolean = false;
  private listeners: Array<(isOnline: boolean) => void> = [];

  constructor() {
    this.init();
  }

  private init() {
    NetInfo.addEventListener(state => {
      const wasOffline = !this.isOnline;
      this.isOnline = state.isConnected ?? false;

      console.log('[Network] Status changed:', {
        isConnected: state.isConnected,
        type: state.type,
      });

      this.listeners.forEach(listener => listener(this.isOnline));

      if (wasOffline && this.isOnline) {
        console.log('[Network] Connection restored, triggering sync');
      }
    });
  }

  getIsOnline(): boolean {
    return this.isOnline;
  }

  addListener(listener: (isOnline: boolean) => void) {
    this.listeners.push(listener);
    listener(this.isOnline);
  }

  removeListener(listener: (isOnline: boolean) => void) {
    this.listeners = this.listeners.filter(l => l !== listener);
  }

  async fetch() {
    const state = await NetInfo.fetch();
    this.isOnline = state.isConnected ?? false;
    return this.isOnline;
  }
}

export const networkService = new NetworkService();


