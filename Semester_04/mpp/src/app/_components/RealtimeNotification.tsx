"use client";

import { useEffect, useState } from "react";
import { useWebSocket } from "./WebSocketProvider";
import { Exercise } from "./CameraContext";

export default function RealtimeNotification() {
  const { lastEvent, isConnected } = useWebSocket();
  const [notification, setNotification] = useState<{
    type: string;
    message: string;
    timestamp: number;
  } | null>(null);

  useEffect(() => {
    if (!lastEvent) return;

    let message = "";

    if (lastEvent.name === "exercise:created") {
      const exercise = lastEvent.data as Exercise;
      message = `New exercise added: ${exercise.name}`;
    } else if (lastEvent.name === "exercise:updated") {
      const exercise = lastEvent.data as Partial<Exercise> & { id: string };
      message = `Exercise updated: ID ${exercise.id.substring(0, 8)}`;
    } else if (lastEvent.name === "exercise:deleted") {
      const { id } = lastEvent.data as { id: string };
      message = `Exercise deleted: ID ${id.substring(0, 8)}`;
    }

    if (message) {
      setNotification({
        type: lastEvent.name,
        message,
        timestamp: Date.now(),
      });

      const timer = setTimeout(() => {
        setNotification(null);
      }, 3000);

      return () => clearTimeout(timer);
    }
  }, [lastEvent]);

  if (!notification) return null;

  return (
    <div className="animate-slide-up fixed bottom-4 right-4 z-50">
      <div className="max-w-xs rounded-lg bg-zinc-800 p-3 shadow-lg">
        <div className="flex items-center gap-2">
          {notification.type.includes("created") && (
            <span className="text-xl text-green-500">+</span>
          )}
          {notification.type.includes("updated") && (
            <span className="text-xl text-yellow-500">↻</span>
          )}
          {notification.type.includes("deleted") && (
            <span className="text-xl text-red-500">×</span>
          )}
          <p className="text-sm">{notification.message}</p>
        </div>
        <div className="mt-1">
          <p className="text-xs text-zinc-500">
            {new Date(notification.timestamp).toLocaleTimeString()}
          </p>
        </div>
      </div>
    </div>
  );
}
