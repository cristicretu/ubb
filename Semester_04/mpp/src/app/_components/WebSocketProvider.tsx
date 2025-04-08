"use client";

import {
  createContext,
  useContext,
  useEffect,
  useState,
  ReactNode,
  useRef,
} from "react";
import { io, Socket } from "socket.io-client";
import { toast } from "sonner";

interface WebSocketContextType {
  socket: Socket | null;
  isConnected: boolean;
  lastEvent: { name: string; data: any } | null;
}

const WebSocketContext = createContext<WebSocketContextType | undefined>(
  undefined,
);

export function WebSocketProvider({ children }: { children: ReactNode }) {
  const [socket, setSocket] = useState<Socket | null>(null);
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [lastEvent, setLastEvent] = useState<{
    name: string;
    data: any;
  } | null>(null);
  const [connectionAttempts, setConnectionAttempts] = useState(0);
  const wasConnectedRef = useRef(false);

  useEffect(() => {
    let socketInstance: Socket | null = null;
    let initializationTimeout: NodeJS.Timeout | null = null;

    const initSocket = async () => {
      try {
        await fetch("/api/socket");

        socketInstance = io({
          path: "/api/socket",
          reconnection: true,
          reconnectionAttempts: 5,
          reconnectionDelay: 1000,
          timeout: 20000,
          autoConnect: true,
          forceNew: connectionAttempts > 2,
          transports: ["polling", "websocket"],
        });

        socketInstance.on("connect", () => {
          console.log("Socket connected:", socketInstance?.id);
          setIsConnected(true);

          if (!wasConnectedRef.current) {
            toast.success("Real-time connection established");
            wasConnectedRef.current = true;
          }
        });

        socketInstance.on("disconnect", () => {
          console.log("Socket disconnected");
          setIsConnected(false);
          wasConnectedRef.current = false;
          toast.error("Real-time connection lost");
        });

        socketInstance.on("connect_error", (err) => {
          console.error("Socket connection error:", err);

          if (wasConnectedRef.current) {
            toast.error("Failed to connect to real-time service");
            wasConnectedRef.current = false;
          }
          setIsConnected(false);

          if (connectionAttempts > 3 && socketInstance) {
            socketInstance.close();

            initializationTimeout = setTimeout(() => {
              setConnectionAttempts((prev) => prev + 1);
            }, 5000);
          }
        });

        socketInstance.on("exercise:created", (data) => {
          console.log("New exercise created:", data);
          setLastEvent({ name: "exercise:created", data });
          toast.info(`New exercise added: ${data.name}`);
        });

        socketInstance.on("exercise:updated", (data) => {
          console.log("Exercise updated:", data);
          setLastEvent({ name: "exercise:updated", data });
        });

        socketInstance.on("exercise:deleted", (data) => {
          console.log("Exercise deleted:", data);
          setLastEvent({ name: "exercise:deleted", data });
        });

        socketInstance.on("data:synced", (data) => {
          console.log("Data synced event received:", data);
          setLastEvent({ name: "data:synced", data });
        });

        setSocket(socketInstance);
      } catch (error) {
        console.error("Failed to initialize socket:", error);

        if (connectionAttempts === 0) {
          toast.error("Failed to initialize real-time service");
        }

        initializationTimeout = setTimeout(() => {
          setConnectionAttempts((prev) => prev + 1);
        }, 5000);
      }
    };

    initSocket();

    return () => {
      if (socketInstance) {
        socketInstance.disconnect();
      }
      if (initializationTimeout) {
        clearTimeout(initializationTimeout);
      }
    };
  }, [connectionAttempts]);

  return (
    <WebSocketContext.Provider value={{ socket, isConnected, lastEvent }}>
      {children}
    </WebSocketContext.Provider>
  );
}

export function useWebSocket() {
  const context = useContext(WebSocketContext);
  if (!context) {
    throw new Error("useWebSocket must be used within a WebSocketProvider");
  }
  return context;
}
