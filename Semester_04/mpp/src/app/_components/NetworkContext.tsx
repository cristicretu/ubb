"use client";

import React, {
  createContext,
  useContext,
  useEffect,
  useState,
  ReactNode,
} from "react";
import { toast } from "sonner";

interface NetworkContextType {
  isOnline: boolean;
  isServerAvailable: boolean;
  checkServerAvailability: () => Promise<boolean>;
}

const NetworkContext = createContext<NetworkContextType | undefined>(undefined);

export function NetworkProvider({ children }: { children: ReactNode }) {
  const [isOnline, setIsOnline] = useState<boolean>(
    typeof window !== "undefined" ? navigator.onLine : true,
  );
  const [isServerAvailable, setIsServerAvailable] = useState<boolean>(true);

  const checkServerAvailability = async (): Promise<boolean> => {
    try {
      const response = await fetch("/api/health", {
        method: "HEAD",

        signal: AbortSignal.timeout(3000),
      });
      const available = response.ok;
      setIsServerAvailable(available);
      return available;
    } catch (error) {
      setIsServerAvailable(false);
      return false;
    }
  };

  useEffect(() => {
    const handleOnline = () => {
      setIsOnline(true);
      toast.success("You are back online");

      checkServerAvailability();
    };

    const handleOffline = () => {
      setIsOnline(false);
      toast.error("You are offline. Changes will be saved locally.");
    };

    window.addEventListener("online", handleOnline);
    window.addEventListener("offline", handleOffline);

    checkServerAvailability();

    let intervalId: NodeJS.Timeout;
    if (isOnline) {
      intervalId = setInterval(checkServerAvailability, 30000);
    }

    return () => {
      window.removeEventListener("online", handleOnline);
      window.removeEventListener("offline", handleOffline);
      if (intervalId) clearInterval(intervalId);
    };
  }, [isOnline]);

  useEffect(() => {
    if (isOnline && !isServerAvailable) {
      toast.error(
        "Server is currently unavailable. Changes will be saved locally.",
      );
    } else if (isOnline && isServerAvailable) {
      const wasOfflineOrServerDown = localStorage.getItem(
        "wasOfflineOrServerDown",
      );
      if (wasOfflineOrServerDown === "true") {
        toast.success("Server connection restored");
        localStorage.removeItem("wasOfflineOrServerDown");
      }
    }

    if (!isOnline || !isServerAvailable) {
      localStorage.setItem("wasOfflineOrServerDown", "true");
    }
  }, [isOnline, isServerAvailable]);

  return (
    <NetworkContext.Provider
      value={{ isOnline, isServerAvailable, checkServerAvailability }}
    >
      {children}
    </NetworkContext.Provider>
  );
}

export function useNetwork() {
  const context = useContext(NetworkContext);
  if (context === undefined) {
    throw new Error("useNetwork must be used within a NetworkProvider");
  }
  return context;
}
