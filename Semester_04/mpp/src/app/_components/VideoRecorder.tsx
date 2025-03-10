"use client";

import { useRef, useState, useEffect } from "react";

interface VideoRecorderProps {
  stream: MediaStream;
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

  // Set up the media recorder when the stream changes
  useEffect(() => {
    if (!stream || stream.getVideoTracks().length === 0) {
      console.error("VideoRecorder: No video tracks in stream");
      return;
    }

    // Clean up any existing recorder
    if (
      mediaRecorderRef.current &&
      mediaRecorderRef.current.state !== "inactive"
    ) {
      try {
        mediaRecorderRef.current.stop();
      } catch (e) {
        console.error("Error stopping existing recorder:", e);
      }
    }

    try {
      // Try to create a media recorder with a supported format
      let recorder: MediaRecorder;

      // Try different MIME types
      const mimeTypes = [
        "video/webm;codecs=vp9,opus",
        "video/webm;codecs=vp8,opus",
        "video/webm",
        "video/mp4",
      ];

      // Find the first supported MIME type
      let supportedType: string | undefined;
      for (const type of mimeTypes) {
        if (MediaRecorder.isTypeSupported(type)) {
          supportedType = type;
          break;
        }
      }

      // Create the recorder with the supported type or default
      if (supportedType) {
        console.log(`VideoRecorder: Using MIME type ${supportedType}`);
        recorder = new MediaRecorder(stream, { mimeType: supportedType });
      } else {
        console.log("VideoRecorder: Using default MIME type");
        recorder = new MediaRecorder(stream);
      }

      // Set up event handlers
      recorder.ondataavailable = (event) => {
        if (event.data && event.data.size > 0) {
          setRecordedChunks((prev) => [...prev, event.data]);
        }
      };

      recorder.onstop = () => {
        if (recordedChunks.length > 0) {
          const blob = new Blob(recordedChunks, {
            type: recordedChunks[0]?.type || "video/webm",
          });
          onRecordingComplete(blob);
          setRecordedChunks([]);
        }
      };

      // Store the recorder reference
      mediaRecorderRef.current = recorder;
    } catch (err) {
      console.error("VideoRecorder: Error creating MediaRecorder:", err);
    }

    // Clean up when stream changes or component unmounts
    return () => {
      if (
        mediaRecorderRef.current &&
        mediaRecorderRef.current.state !== "inactive"
      ) {
        try {
          mediaRecorderRef.current.stop();
        } catch (e) {
          console.error("Error stopping recorder during cleanup:", e);
        }
      }
    };
  }, [stream]);

  // Control recording state
  useEffect(() => {
    if (!mediaRecorderRef.current) return;

    try {
      if (isRecording) {
        if (mediaRecorderRef.current.state === "inactive") {
          console.log("VideoRecorder: Starting recording");
          setRecordedChunks([]);
          mediaRecorderRef.current.start(100); // Collect data every 100ms
        }
      } else {
        if (mediaRecorderRef.current.state === "recording") {
          console.log("VideoRecorder: Stopping recording");
          mediaRecorderRef.current.stop();
        }
      }
    } catch (err) {
      console.error("VideoRecorder: Error controlling recording:", err);
    }
  }, [isRecording]);

  // No UI needed
  return null;
}
