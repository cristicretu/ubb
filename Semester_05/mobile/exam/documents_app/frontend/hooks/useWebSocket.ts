import { useEffect, useState, useRef } from 'react';
import { WS_URL } from '../config';

interface UseWebSocketOptions {
  onMessage?: (message: any) => void;
}

export function useWebSocket({ onMessage }: UseWebSocketOptions = {}) {
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<any>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const ws = new WebSocket(WS_URL);

    ws.onopen = () => setIsConnected(true);
    ws.onclose = () => setIsConnected(false);
    ws.onerror = () => setIsConnected(false);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        setLastMessage(data);
        onMessage?.(data);
      } catch {
        setLastMessage(event.data);
        onMessage?.(event.data);
      }
    };

    wsRef.current = ws;

    return () => {
      wsRef.current?.close();
      wsRef.current = null;
    };
  }, [onMessage]);

  return { isConnected, lastMessage };
}
