"use client";

import { useState } from "react";
import { useCameraContext } from "./CameraContext";
import { useWebSocket } from "./WebSocketProvider";

export default function ExerciseGeneratorControl() {
  const { startExerciseGenerator, stopExerciseGenerator, isGeneratorRunning } =
    useCameraContext();

  const { isConnected } = useWebSocket();
  const [interval, setInterval] = useState<number>(30);
  const [isLoading, setIsLoading] = useState<boolean>(false);

  const handleStart = async () => {
    setIsLoading(true);
    await startExerciseGenerator(interval * 1000);
    setIsLoading(false);
  };

  const handleStop = async () => {
    setIsLoading(true);
    await stopExerciseGenerator();
    setIsLoading(false);
  };

  return (
    <div className="rounded-lg bg-zinc-900 p-4 shadow-lg">
      <h3 className="mb-3 text-lg font-semibold">
        Real-Time Exercise Generator
      </h3>

      <div className="mb-4 flex items-center gap-2">
        <div
          className={`h-3 w-3 rounded-full ${isConnected ? "bg-green-500" : "bg-red-500"}`}
        ></div>
        <span className="text-sm">
          {isConnected
            ? "Connected to real-time service"
            : "Not connected to real-time service"}
        </span>
      </div>

      <div className="mb-4">
        <label className="mb-1 block text-sm">
          Generation interval (seconds)
        </label>
        <input
          type="number"
          min="5"
          max="60"
          value={interval}
          onChange={(e) => setInterval(Number(e.target.value))}
          disabled={isGeneratorRunning || isLoading}
          className="w-full rounded bg-zinc-800 px-3 py-2 text-white"
        />
      </div>

      <div className="flex gap-2">
        {!isGeneratorRunning ? (
          <button
            onClick={handleStart}
            disabled={!isConnected || isLoading}
            className="rounded bg-blue-600 px-4 py-2 font-medium text-white hover:bg-blue-700 disabled:cursor-not-allowed disabled:opacity-50"
          >
            {isLoading ? "Starting..." : "Start Generator"}
          </button>
        ) : (
          <button
            onClick={handleStop}
            disabled={!isConnected || isLoading}
            className="rounded bg-red-600 px-4 py-2 font-medium text-white hover:bg-red-700 disabled:cursor-not-allowed disabled:opacity-50"
          >
            {isLoading ? "Stopping..." : "Stop Generator"}
          </button>
        )}
      </div>

      {isGeneratorRunning && (
        <p className="mt-3 text-sm text-green-500">
          Generator is running. New exercises will appear automatically in your
          list.
        </p>
      )}
    </div>
  );
}
