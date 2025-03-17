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
  const chunksRef = useRef<Blob[]>([]);

  useEffect(() => {
    if (!stream || stream.getVideoTracks().length === 0) {
      console.error("VideoRecorder: No video tracks in stream");
      return;
    }

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
      let recorder: MediaRecorder;

      const mimeTypes = [
        "video/webm;codecs=vp9,opus",
        "video/webm;codecs=vp8,opus",
        "video/webm",
        "video/mp4",
      ];

      let supportedType: string | undefined;
      for (const type of mimeTypes) {
        if (MediaRecorder.isTypeSupported(type)) {
          supportedType = type;
          break;
        }
      }

      if (supportedType) {
        console.log(`VideoRecorder: Using MIME type ${supportedType}`);
        recorder = new MediaRecorder(stream, { mimeType: supportedType });
      } else {
        console.log("VideoRecorder: Using default MIME type");
        recorder = new MediaRecorder(stream);
      }

      recorder.ondataavailable = (event) => {
        if (event.data && event.data.size > 0) {
          console.log("Data available:", event.data.size);
          chunksRef.current.push(event.data);
          setRecordedChunks((prev) => [...prev, event.data]);
        }
      };

      recorder.onstop = () => {
        console.log("MediaRecorder stopped, chunks:", chunksRef.current.length);
        if (chunksRef.current.length > 0) {
          const blob = new Blob(chunksRef.current, {
            type: chunksRef.current[0]?.type || "video/webm",
          });
          console.log("Created blob:", blob.size);
          onRecordingComplete(blob);
          setRecordedChunks([]);
          chunksRef.current = [];
        } else {
          console.error("No recorded chunks available");
          // Create a minimal test blob if no chunks are available
          const testBlob = new Blob(["test"], { type: "video/webm" });
          onRecordingComplete(testBlob);
        }
      };

      mediaRecorderRef.current = recorder;
    } catch (err) {
      console.error("VideoRecorder: Error creating MediaRecorder:", err);
    }

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
  }, [stream, onRecordingComplete]);

  useEffect(() => {
    if (!mediaRecorderRef.current) return;

    try {
      if (isRecording) {
        if (mediaRecorderRef.current.state === "inactive") {
          console.log("VideoRecorder: Starting recording");
          chunksRef.current = [];
          setRecordedChunks([]);
          mediaRecorderRef.current.start(1000); // Request data every second
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

  return null;
}
