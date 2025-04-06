"use client";

import { useRef, useState, useEffect, useCallback } from "react";
import {
  Play,
  Square,
  RefreshCw,
  AlertCircle,
  Camera as CameraIcon,
  Video,
  VideoOff,
  Upload,
} from "lucide-react";
import VideoRecorder from "./VideoRecorder";
import { useCameraContext } from "./CameraContext";
import ExerciseForm from "./ExerciseForm";
import UploadVideo from "./UploadVideo";
import Link from "next/link";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { Toaster } from "~/components/ui/sonner";
import { toast } from "sonner";

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
  const [mediaStream, setMediaStream] = useState<MediaStream | null>(null);
  const { addRecordedVideo, exercises, addExercise } = useCameraContext();
  const [showExerciseForm, setShowExerciseForm] = useState(false);
  const [currentVideoUrl, setCurrentVideoUrl] = useState<string | null>(null);
  const [showUploadMode, setShowUploadMode] = useState(false);

  const [chartKey, setChartKey] = useState(0);

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
          setMediaStream(stream);
        }
      } catch (err) {
        console.error("Failed to get camera access:", err);
        setError(
          `Camera access error: ${err instanceof Error ? err.message : String(err)}`,
        );
        setIsStreaming(false);
      }
    }

    if (!showUploadMode) {
      setupCamera();
    }

    return () => {
      if (videoRef.current?.srcObject) {
        const stream = videoRef.current.srcObject as MediaStream;
        stream.getTracks().forEach((track) => track.stop());
      }
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
    };
  }, [showUploadMode]);

  const toggleRecording = () => {
    if (isRecording) {
      setIsRecording(false);
      if (recordingStartTime) {
        const duration = Math.round((Date.now() - recordingStartTime) / 1000);
        setRecordingStartTime(null);
        setRecordingDuration(duration);

        if (timerRef.current) {
          clearInterval(timerRef.current);
          timerRef.current = null;
        }
      }
    } else {
      setIsRecording(true);
      setRecordingStartTime(Date.now());
      setRecordingDuration(0);

      timerRef.current = setInterval(() => {
        if (recordingStartTime !== null) {
          const elapsed = Math.round((Date.now() - recordingStartTime) / 1000);
          setRecordingDuration(elapsed);
        } else {
          const elapsed = Math.round((Date.now() - Date.now()) / 1000);
          setRecordingDuration(elapsed);
        }
      }, 1000);
    }
  };

  const handleRecordingComplete = async (blob: Blob) => {
    const videoUrl = URL.createObjectURL(blob);
    setCurrentVideoUrl(videoUrl);

    try {
      const formData = new FormData();
      const timestamp = new Date().toISOString().replace(/[:.]/g, "-");
      const filename = `recording-${timestamp}.webm`;
      const file = new File([blob], filename, {
        type: blob.type || "video/webm",
      });
      formData.append("file", file);

      console.log("Recording blob details:", {
        type: blob.type,
        size: blob.size,
        filename,
      });

      toast.loading("Saving recording...", { id: "save-recording" });

      const response = await fetch("/api/upload", {
        method: "POST",
        body: formData,
      });

      if (!response.ok) {
        const errorText = await response.text().catch(() => "Unknown error");
        throw new Error(
          `Upload failed with status: ${response.status} - ${errorText}`,
        );
      }

      let data;
      try {
        data = await response.json();
        console.log("Upload response:", data);
      } catch (error) {
        console.error("Failed to parse upload response:", error);
        throw new Error("Invalid server response");
      }

      if (data.success && data.fileUrl) {
        toast.success("Recording saved to server", { id: "save-recording" });

        // Use the absolute URL from the server directly
        let serverVideoUrl = data.fileUrl;

        // Debug additional info about the URL
        console.log("Server video URL details:", {
          url: serverVideoUrl,
          origin: window.location.origin,
          isAbsolute: serverVideoUrl.startsWith("http"),
          urlObj: serverVideoUrl.startsWith("http")
            ? new URL(serverVideoUrl)
            : null,
        });

        console.log("Final video URL for exercise:", serverVideoUrl);

        URL.revokeObjectURL(videoUrl);

        setCurrentVideoUrl(serverVideoUrl);

        addRecordedVideo(serverVideoUrl);
        setShowExerciseForm(true);
      } else {
        throw new Error(data.error || "Upload failed - no file URL returned");
      }
    } catch (error) {
      console.error("Error saving recording:", error);
      toast.error(
        `Failed to save recording to server: ${error instanceof Error ? error.message : "Unknown error"}`,
        { id: "save-recording" },
      );
      addRecordedVideo(videoUrl);
      setShowExerciseForm(true);
    }
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const toggleMode = () => {
    // If switching to upload mode, stop the camera stream
    if (!showUploadMode && videoRef.current?.srcObject) {
      const stream = videoRef.current.srcObject as MediaStream;
      stream.getTracks().forEach((track) => track.stop());
      setIsStreaming(false);
      setMediaStream(null);
    }

    setShowUploadMode(!showUploadMode);
  };

  return (
    <div className="flex min-h-screen flex-col items-center justify-center p-4">
      <div className="flex w-full max-w-5xl flex-col">
        <div className="mb-4 flex justify-end gap-2">
          <button
            onClick={toggleMode}
            className="flex items-center gap-2 rounded-md bg-zinc-800 px-3 py-2 text-sm"
          >
            {showUploadMode ? (
              <>
                <CameraIcon size={16} />
                Switch to Camera
              </>
            ) : (
              <>
                <Upload size={16} />
                Switch to Upload
              </>
            )}
          </button>
        </div>

        {error && (
          <div className="mb-4 rounded bg-red-100 p-4 text-red-700">
            <p>{error}</p>
          </div>
        )}

        {showUploadMode ? (
          <UploadVideo />
        ) : (
          <>
            <div className="relative mb-6 overflow-hidden rounded-lg">
              <video
                ref={videoRef}
                autoPlay
                playsInline
                muted
                className="h-full w-full object-cover"
              />

              {isStreaming && (
                <>
                  <div className="absolute right-2 top-2 rounded-full bg-green-500 px-3 py-1 text-xs font-medium text-white">
                    Live
                  </div>

                  {/* {isRecording && (
                    <div className="absolute left-2 top-2 flex items-center gap-2 rounded-full bg-red-500 px-3 py-1 text-xs font-medium text-white">
                      <span className="h-2 w-2 animate-pulse rounded-full bg-white"></span>
                      Recording {formatDuration(recordingDuration)}
                    </div>
                  )} */}

                  <button
                    onClick={toggleRecording}
                    disabled={!isStreaming}
                    className="absolute bottom-2 left-1/2 flex -translate-x-1/2 items-center justify-center gap-2"
                  >
                    <div
                      className={`h-12 w-12 rounded-full ${
                        isRecording ? "animate-pulse bg-red-500" : "bg-white"
                      }`}
                    />
                  </button>
                </>
              )}
            </div>

            <Dialog open={showExerciseForm} onOpenChange={setShowExerciseForm}>
              <DialogContent className="sm:max-w-md">
                <DialogHeader>
                  <DialogTitle>Save Exercise</DialogTitle>
                  <DialogDescription>
                    Review and save your recorded exercise.
                  </DialogDescription>
                </DialogHeader>
                {currentVideoUrl && (
                  <ExerciseForm
                    videoUrl={currentVideoUrl}
                    duration={recordingDuration}
                    onCancel={() => setShowExerciseForm(false)}
                    onSave={() => {
                      setShowExerciseForm(false);
                      setCurrentVideoUrl(null);
                      toast.success("Exercise saved successfully");
                    }}
                  />
                )}
              </DialogContent>
            </Dialog>

            {mediaStream && (
              <VideoRecorder
                stream={mediaStream}
                isRecording={isRecording}
                onRecordingComplete={handleRecordingComplete}
              />
            )}
          </>
        )}
      </div>
    </div>
  );
}
