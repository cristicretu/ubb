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

    // First, ensure the Socket.io server is initialized
    const initSocket = async () => {
      try {
        // Call the API route to initialize the Socket.io server
        await fetch("/api/socket");

        // Then create the socket connection with more robust options
        socketInstance = io({
          path: "/api/socket",
          reconnection: true,
          reconnectionAttempts: 5,
          reconnectionDelay: 1000,
          timeout: 20000,
          autoConnect: true,
          forceNew: connectionAttempts > 2, // Force new connection after multiple attempts
          transports: ["polling", "websocket"], // Start with polling, upgrade to websocket
        });

        socketInstance.on("connect", () => {
          console.log("Socket connected:", socketInstance?.id);
          setIsConnected(true);
          // Only show toast on first successful connection or after a disconnect
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
          // Only show the toast error once, not for every retry
          if (wasConnectedRef.current) {
            toast.error("Failed to connect to real-time service");
            wasConnectedRef.current = false;
          }
          setIsConnected(false);

          // After multiple failures, try to recreate the socket
          if (connectionAttempts > 3 && socketInstance) {
            socketInstance.close();

            // Try again after a delay
            initializationTimeout = setTimeout(() => {
              setConnectionAttempts((prev) => prev + 1);
            }, 5000);
          }
        });

        // Listen for exercise events
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

        setSocket(socketInstance);
      } catch (error) {
        console.error("Failed to initialize socket:", error);
        // Don't show a toast on every retry
        if (connectionAttempts === 0) {
          toast.error("Failed to initialize real-time service");
        }

        // Try again after a delay
        initializationTimeout = setTimeout(() => {
          setConnectionAttempts((prev) => prev + 1);
        }, 5000);
      }
    };

    initSocket();

    // Clean up on unmount
    return () => {
      if (socketInstance) {
        socketInstance.disconnect();
      }
      if (initializationTimeout) {
        clearTimeout(initializationTimeout);
      }
    };
  }, [connectionAttempts]); // Removed isConnected from dependencies

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
