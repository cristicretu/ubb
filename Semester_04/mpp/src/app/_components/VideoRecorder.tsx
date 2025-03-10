"use client";

import { useRef, useState, useEffect } from "react";

interface VideoRecorderProps {
  stream: MediaStream | null;
  isRecording: boolean;
  onRecordingComplete: (blob: Blob) => void;
}

export default function VideoRecorder({
  stream,
  isRecording,
  onRecordingComplete,
}: VideoRecorderProps) {
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const [recordedChunks, setRecordedChunks] = useState<Blob[]>([]);
  const [isBrowser, setIsBrowser] = useState(false);
  const [recorderReady, setRecorderReady] = useState(false);

  // Check if we're in the browser
  useEffect(() => {
    setIsBrowser(true);
  }, []);

  // Set up media recorder when stream changes
  useEffect(() => {
    // Only run this effect in the browser
    if (!isBrowser || !stream) return;

    console.log("VideoRecorder: Setting up media recorder with stream");

    // Reset state
    setRecorderReady(false);
    setRecordedChunks([]);

    // Check if stream has video tracks
    if (stream.getVideoTracks().length === 0) {
      console.error("VideoRecorder: Stream has no video tracks");
      return;
    }

    // Set up media recorder
    try {
      let options;
      try {
        options = { mimeType: "video/webm;codecs=vp9,opus" };
        mediaRecorderRef.current = new MediaRecorder(stream, options);
        console.log("VideoRecorder: Using video/webm;codecs=vp9,opus");
      } catch (e) {
        try {
          // Fallback for browsers that don't support the above options
          options = { mimeType: "video/webm" };
          mediaRecorderRef.current = new MediaRecorder(stream, options);
          console.log("VideoRecorder: Using video/webm");
        } catch (e) {
          // Final fallback
          mediaRecorderRef.current = new MediaRecorder(stream);
          console.log("VideoRecorder: Using default MIME type");
        }
      }

      // Set up event handlers
      if (mediaRecorderRef.current) {
        mediaRecorderRef.current.ondataavailable = handleDataAvailable;
        mediaRecorderRef.current.onstop = handleStop;
        mediaRecorderRef.current.onerror = (event) => {
          console.error("MediaRecorder error:", event);
        };

        setRecorderReady(true);
        console.log("VideoRecorder: Media recorder ready");
      }
    } catch (e) {
      console.error(
        "VideoRecorder: MediaRecorder is not supported by this browser.",
        e,
      );
      return;
    }

    return () => {
      if (mediaRecorderRef.current) {
        if (mediaRecorderRef.current.state !== "inactive") {
          console.log("VideoRecorder: Stopping recorder on cleanup");
          try {
            mediaRecorderRef.current.stop();
          } catch (e) {
            console.error("Error stopping media recorder:", e);
          }
        }
      }
    };
  }, [isBrowser, stream]);

  // Start or stop recording based on isRecording prop
  useEffect(() => {
    if (!isBrowser || !mediaRecorderRef.current || !stream || !recorderReady)
      return;

    try {
      if (isRecording) {
        console.log("VideoRecorder: Starting recording");
        setRecordedChunks([]);
        mediaRecorderRef.current.start(100); // Collect data every 100ms
      } else if (mediaRecorderRef.current.state !== "inactive") {
        console.log("VideoRecorder: Stopping recording");
        mediaRecorderRef.current.stop();
      }
    } catch (err) {
      console.error("VideoRecorder: Error controlling media recorder:", err);
    }
  }, [isRecording, stream, isBrowser, recorderReady]);

  const handleDataAvailable = (event: BlobEvent) => {
    if (event.data && event.data.size > 0) {
      console.log("VideoRecorder: Data available, size:", event.data.size);
      setRecordedChunks((prev) => [...prev, event.data]);
    }
  };

  const handleStop = () => {
    console.log(
      "VideoRecorder: Recording stopped, chunks:",
      recordedChunks.length,
    );

    if (recordedChunks.length > 0) {
      try {
        const blob = new Blob(recordedChunks, {
          type: recordedChunks[0]?.type || "video/webm",
        });
        console.log("VideoRecorder: Created blob, size:", blob.size);
        onRecordingComplete(blob);
      } catch (err) {
        console.error(
          "VideoRecorder: Error creating blob from recorded chunks:",
          err,
        );
      }
    }
  };

  return null; // This is a logic component, no UI
}
