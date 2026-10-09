"use client";

import { useState, useEffect, useRef } from "react";

interface VideoPreviewProps {
  src: string;
  className?: string;
  height?: string;
  width?: string;
  autoPlay?: boolean;
  controls?: boolean;
  loop?: boolean;
  muted?: boolean;
  onLoadedMetadata?: (duration: number) => void;
}

export default function VideoPreview({
  src,
  className = "",
  height = "100%",
  width = "100%",
  autoPlay = false,
  controls = true,
  loop = false,
  muted = false,
  onLoadedMetadata,
}: VideoPreviewProps) {
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    setIsLoading(true);
    setError(null);
  }, [src]);

  const handleLoadedMetadata = () => {
    setIsLoading(false);

    if (videoRef.current && onLoadedMetadata) {
      onLoadedMetadata(videoRef.current.duration);
    }
  };

  const handleError = () => {
    setIsLoading(false);
    setError("Failed to load video. The file may be corrupt or unsupported.");
  };

  return (
    <div className={`relative ${className}`} style={{ height, width }}>
      {isLoading && (
        <div className="absolute inset-0 flex items-center justify-center bg-black bg-opacity-50">
          <div className="h-10 w-10 animate-spin rounded-full border-4 border-blue-500 border-t-transparent"></div>
        </div>
      )}

      {error && (
        <div className="absolute inset-0 flex items-center justify-center bg-black bg-opacity-50">
          <div className="p-4 text-center">
            <svg
              className="mx-auto mb-2 h-10 w-10 text-red-500"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
              />
            </svg>
            <p className="text-sm text-white">{error}</p>
          </div>
        </div>
      )}

      <video
        ref={videoRef}
        className="h-full w-full bg-black object-contain"
        src={src}
        autoPlay={autoPlay}
        controls={controls}
        loop={loop}
        muted={muted}
        playsInline
        onLoadedMetadata={handleLoadedMetadata}
        onError={handleError}
      />
    </div>
  );
}
