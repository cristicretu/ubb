"use client";

import { useEffect, useState } from "react";
import PageLayout from "../_components/PageLayout";
import { useCameraContext } from "../_components/CameraContext";
import Link from "next/link";

interface RecordingEntry {
  date: string;
  duration: number;
}

export default function Gallery() {
  const { recordedVideos } = useCameraContext();
  const [recordings, setRecordings] = useState<RecordingEntry[]>([]);

  useEffect(() => {
    const storedRecordings = localStorage.getItem("recordings");
    if (storedRecordings) {
      try {
        setRecordings(JSON.parse(storedRecordings));
      } catch (error) {
        console.error("Error parsing recordings from localStorage:", error);
      }
    }
  }, []);

  const formatDate = (dateStr: string): string => {
    const date = new Date(dateStr);
    return new Intl.DateTimeFormat("en-US", {
      month: "short",
      day: "2-digit",
      year: "numeric",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: true,
    }).format(date);
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const clearAllRecordings = () => {
    if (window.confirm("Are you sure you want to clear all recordings?")) {
      localStorage.removeItem("recordings");
      setRecordings([]);
    }
  };

  return (
    <PageLayout>
      <div className="container mx-auto px-4 py-8">
        <div className="mb-6 flex items-center justify-between">
          <h1 className="text-3xl font-bold">Gallery</h1>
          <div className="flex gap-4">
            <Link
              href="/"
              className="rounded-md bg-zinc-800 px-4 py-2 text-sm text-white hover:bg-zinc-700"
            >
              Back to Camera
            </Link>
            {recordings.length > 0 && (
              <button
                onClick={clearAllRecordings}
                className="rounded-md bg-red-600 px-4 py-2 text-sm text-white hover:bg-red-700"
              >
                Clear All
              </button>
            )}
          </div>
        </div>

        {recordings.length > 0 ? (
          <div>
            <h2 className="mb-4 text-xl font-semibold">Your Recordings</h2>
            <div className="rounded-lg bg-zinc-800 p-1">
              <table className="w-full">
                <thead>
                  <tr className="border-b border-zinc-700">
                    <th className="p-3 text-left text-sm font-medium">Date</th>
                    <th className="p-3 text-left text-sm font-medium">
                      Duration
                    </th>
                  </tr>
                </thead>
                <tbody>
                  {recordings.map((recording, index) => (
                    <tr
                      key={index}
                      className="border-b border-zinc-700 last:border-b-0"
                    >
                      <td className="p-3 text-sm">
                        {formatDate(recording.date)}
                      </td>
                      <td className="p-3 text-sm">
                        {formatDuration(recording.duration)}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        ) : (
          <div className="flex flex-col items-center justify-center py-20">
            <div className="max-w-md rounded-lg bg-zinc-900 p-8 text-center">
              <h2 className="mb-2 text-xl font-semibold">No recordings yet</h2>
              <p className="mb-4 text-zinc-400">
                Your recordings will appear here. Go to the Camera page to start
                recording.
              </p>
              <Link
                href="/"
                className="inline-block rounded-md bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
              >
                Go to Camera
              </Link>
            </div>
          </div>
        )}

        {recordedVideos.length > 0 && (
          <div className="mt-8">
            <h2 className="mb-4 text-xl font-semibold">Recorded Videos</h2>
            <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
              {recordedVideos.map((videoSrc, index) => (
                <div
                  key={index}
                  className="overflow-hidden rounded-lg bg-zinc-900"
                >
                  <video
                    src={videoSrc}
                    controls
                    className="w-full rounded-lg"
                    playsInline
                  />
                  <div className="p-3">
                    <p className="text-sm text-zinc-400">
                      Recording {index + 1}
                    </p>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>
    </PageLayout>
  );
}
