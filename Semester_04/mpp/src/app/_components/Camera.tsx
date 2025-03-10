"use client";

import { useRef, useState, useEffect } from "react";
import {
  Play,
  Square,
  RefreshCw,
  AlertCircle,
  Camera as CameraIcon,
  Video,
  VideoOff,
} from "lucide-react";
import VideoRecorder from "./VideoRecorder";
import { useCameraContext } from "./CameraContext";
import Link from "next/link";

interface RecordingEntry {
  date: string;
  duration: number; // in seconds
}

export default function CameraTest() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isRecording, setIsRecording] = useState(false);
  const [recordingStartTime, setRecordingStartTime] = useState<number | null>(
    null,
  );
  const [recordingDuration, setRecordingDuration] = useState(0);
  const timerRef = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    async function setupCamera() {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({
          video: true,
          audio: false,
        });

        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          setIsStreaming(true);
          setError(null);
        }
      } catch (err) {
        console.error("Failed to get camera access:", err);
        setError(
          `Camera access error: ${err instanceof Error ? err.message : String(err)}`,
        );
        setIsStreaming(false);
      }
    }

    setupCamera();

    return () => {
      // Cleanup
      if (videoRef.current?.srcObject) {
        const stream = videoRef.current.srcObject as MediaStream;
        stream.getTracks().forEach((track) => track.stop());
      }
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
    };
  }, []);

  const toggleRecording = () => {
    if (isRecording) {
      // Stop recording
      setIsRecording(false);
      if (recordingStartTime) {
        const duration = Math.round((Date.now() - recordingStartTime) / 1000);

        // Save to localStorage
        const recordingEntry: RecordingEntry = {
          date: new Date().toISOString(),
          duration: duration,
        };

        // Get existing recordings from localStorage
        const existingRecordings = JSON.parse(
          localStorage.getItem("recordings") || "[]",
        );

        // Add new recording
        localStorage.setItem(
          "recordings",
          JSON.stringify([recordingEntry, ...existingRecordings]),
        );

        // Reset state
        setRecordingStartTime(null);
        setRecordingDuration(0);

        // Clear timer
        if (timerRef.current) {
          clearInterval(timerRef.current);
          timerRef.current = null;
        }
      }
    } else {
      // Start recording
      setIsRecording(true);
      setRecordingStartTime(Date.now());

      // Start timer to update recording duration
      timerRef.current = setInterval(() => {
        if (recordingStartTime) {
          const elapsed = Math.round((Date.now() - recordingStartTime) / 1000);
          setRecordingDuration(elapsed);
        }
      }, 1000);
    }
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  return (
    <div className="flex min-h-screen flex-col items-center justify-center p-4">
      <div className="flex w-full max-w-3xl flex-col">
        <div className="mb-6 flex items-center justify-between">
          <h1 className="text-2xl font-bold">Camera Test Page</h1>
          <Link
            href="/gallery"
            className="rounded-md bg-zinc-800 px-4 py-2 text-sm text-white hover:bg-zinc-700"
          >
            View Gallery
          </Link>
        </div>

        {error && (
          <div className="mb-4 rounded bg-red-100 p-4 text-red-700">
            <p>{error}</p>
          </div>
        )}

        <div className="relative mb-6 overflow-hidden rounded-lg border border-gray-300">
          <video
            ref={videoRef}
            autoPlay
            playsInline
            muted
            className="h-[60vh] w-full object-cover"
          />

          {isStreaming && (
            <>
              <div className="absolute right-2 top-2 rounded-full bg-green-500 px-3 py-1 text-xs font-medium text-white">
                Live
              </div>

              {isRecording && (
                <div className="absolute left-2 top-2 flex items-center gap-2 rounded-full bg-red-500 px-3 py-1 text-xs font-medium text-white">
                  <span className="h-2 w-2 animate-pulse rounded-full bg-white"></span>
                  Recording {formatDuration(recordingDuration)}
                </div>
              )}
            </>
          )}
        </div>

        <div className="flex justify-center gap-4">
          <button
            onClick={toggleRecording}
            disabled={!isStreaming}
            className={`flex items-center gap-2 rounded-md px-4 py-2 font-medium text-white ${
              isRecording
                ? "bg-red-600 hover:bg-red-700"
                : "bg-blue-600 hover:bg-blue-700"
            } ${!isStreaming && "cursor-not-allowed opacity-50"}`}
          >
            {isRecording ? (
              <>
                <Square size={18} /> Stop Recording
              </>
            ) : (
              <>
                <Video size={18} /> Start Recording
              </>
            )}
          </button>
        </div>
      </div>
    </div>
  );
}
