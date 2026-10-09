const SERVER_HEALTH_URL = 'http://localhost:3000/health';

export class ServerConnectionService {
  private isServerReachable: boolean = false;
  private listeners: Array<(isReachable: boolean) => void> = [];
  private checkInterval: NodeJS.Timeout | null = null;

  startMonitoring() {
    this.checkServerHealth();

    this.checkInterval = setInterval(() => {
      this.checkServerHealth();
    }, 10000);

    console.log('[ServerConnection] Started monitoring');
  }

  stopMonitoring() {
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
      this.checkInterval = null;
    }
    console.log('[ServerConnection] Stopped monitoring');
  }

  private async checkServerHealth() {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 3000);

      const response = await fetch(SERVER_HEALTH_URL, {
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      const wasUnreachable = !this.isServerReachable;
      this.isServerReachable = response.ok;

      if (wasUnreachable && this.isServerReachable) {
        console.log('[ServerConnection] Server is now reachable');
      } else if (!wasUnreachable && !this.isServerReachable) {
        console.log('[ServerConnection] Server is now unreachable');
      }

      this.notifyListeners();
    } catch (error) {
      const wasReachable = this.isServerReachable;
      this.isServerReachable = false;

      if (wasReachable) {
        console.log('[ServerConnection] Server unreachable:', error instanceof Error ? error.message : 'timeout');
      }

      this.notifyListeners();
    }
  }

  getIsServerReachable(): boolean {
    return this.isServerReachable;
  }

  addListener(listener: (isReachable: boolean) => void) {
    this.listeners.push(listener);
    listener(this.isServerReachable);
  }

  removeListener(listener: (isReachable: boolean) => void) {
    this.listeners = this.listeners.filter(l => l !== listener);
  }

  private notifyListeners() {
    this.listeners.forEach(listener => listener(this.isServerReachable));
  }

  async checkNow(): Promise<boolean> {
    await this.checkServerHealth();
    return this.isServerReachable;
  }
}

export const serverConnectionService = new ServerConnectionService();
