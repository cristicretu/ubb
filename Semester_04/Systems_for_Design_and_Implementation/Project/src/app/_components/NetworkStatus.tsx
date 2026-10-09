"use client";

import React from "react";
import { useNetwork } from "./NetworkContext";
import { useCameraContext } from "./CameraContext";

export default function NetworkStatus() {
  const { isOnline, isServerAvailable } = useNetwork();
  const { pendingOperationsCount, syncPendingOperations } = useCameraContext();

  if (isOnline && isServerAvailable && pendingOperationsCount === 0) {
    return null;
  }

  return (
    <div className="fixed bottom-4 left-4 z-50 rounded-lg border border-gray-200 bg-white p-4 shadow-lg dark:border-gray-700 dark:bg-gray-800">
      <div className="flex flex-col space-y-2">
        {!isOnline && (
          <div className="flex items-center text-red-500">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="mr-2 h-5 w-5"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M18.364 5.636a9 9 0 010 12.728m-4.95-4.95a3 3 0 104.243-4.243m-4.243 4.243L5.636 5.636M16.243 7.757a6 6 0 00-8.486 8.486m8.486-8.486L5.636 18.364"
              />
            </svg>
            <span>You are offline</span>
          </div>
        )}

        {isOnline && !isServerAvailable && (
          <div className="flex items-center text-amber-500">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="mr-2 h-5 w-5"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z"
              />
            </svg>
            <span>Server is unavailable</span>
          </div>
        )}

        {pendingOperationsCount > 0 && (
          <div className="flex items-center text-blue-500">
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="mr-2 h-5 w-5"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"
              />
            </svg>
            <span>
              {pendingOperationsCount} pending operation
              {pendingOperationsCount !== 1 ? "s" : ""}
            </span>
          </div>
        )}

        {isOnline && isServerAvailable && pendingOperationsCount > 0 && (
          <button
            onClick={() => syncPendingOperations()}
            className="mt-2 rounded bg-blue-500 px-4 py-2 text-white transition-colors hover:bg-blue-600"
          >
            Sync now
          </button>
        )}
      </div>
    </div>
  );
}
