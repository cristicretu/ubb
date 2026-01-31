import { useEffect, useState, useRef } from 'react';

// WebSocket URL - configurable, defaults to Android emulator address
const WS_URL = process.env.EXPO_PUBLIC_WS_URL || 'ws://10.0.2.2:3000';

interface UseWebSocketOptions {
  onMessage?: (message: any) => void;
}

interface UseWebSocketReturn {
  isConnected: boolean;
  lastMessage: any | null;
}

export function useWebSocket({ onMessage }: UseWebSocketOptions = {}): UseWebSocketReturn {
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<any | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Create WebSocket connection
    const ws = new WebSocket(WS_URL);

    ws.onopen = () => {
      console.log('WebSocket connected');
      setIsConnected(true);
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        setLastMessage(data);
        if (onMessage) {
          onMessage(data);
        }
      } catch (error) {
        console.error('Error parsing WebSocket message:', error);
        setLastMessage(event.data);
        if (onMessage) {
          onMessage(event.data);
        }
      }
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
      setIsConnected(false);
    };

    ws.onclose = () => {
      console.log('WebSocket disconnected');
      setIsConnected(false);
    };

    wsRef.current = ws;

    // Cleanup on unmount
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [onMessage]);

  return { isConnected, lastMessage };
}
