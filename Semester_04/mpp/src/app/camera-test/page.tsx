"use client";

import { useRef, useState, useEffect } from "react";
import Link from "next/link";

export default function CameraTest() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [error, setError] = useState<string | null>(null);

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
      if (videoRef.current?.srcObject) {
        const stream = videoRef.current.srcObject as MediaStream;
        stream.getTracks().forEach((track) => track.stop());
      }
    };
  }, []);

  return (
    <div className="flex min-h-screen flex-col items-center justify-center p-4">
      <h1 className="mb-6 text-2xl font-bold">Camera Test Page</h1>

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
          <div className="absolute right-2 top-2 rounded-full bg-green-500 px-3 py-1 text-xs font-medium text-white">
            Live
          </div>
        )}
      </div>

      <div className="flex gap-4">
        <Link
          href="/"
          className="rounded bg-blue-500 px-4 py-2 font-medium text-white hover:bg-blue-600"
        >
          Back to Main App
        </Link>
      </div>
    </div>
  );
}
