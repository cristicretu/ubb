"use client";

import { useRef, useEffect } from "react";

interface VideoPlayerProps {
  src: string;
  autoPlay?: boolean;
  controls?: boolean;
  className?: string;
}

export default function VideoPlayer({
  src,
  autoPlay = false,
  controls = true,
  className = "",
}: VideoPlayerProps) {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current && autoPlay) {
      void videoRef.current.play().catch((error) => {
        console.error("Error auto-playing video:", error);
      });
    }
  }, [src, autoPlay]);

  return (
    <video
      ref={videoRef}
      src={src}
      controls={controls}
      className={`w-full rounded-lg ${className}`}
      playsInline
    />
  );
}
