"use client";

import { useRef, useState, useEffect } from "react";
import {
  Play,
  Square,
  RefreshCw,
  AlertCircle,
  Camera as CameraIcon,
} from "lucide-react";
import VideoRecorder from "./VideoRecorder";
import { useCameraContext } from "./CameraContext";

export default function Camera() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [isRecording, setIsRecording] = useState(false);
  const [stream, setStream] = useState<MediaStream | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [isBrowser, setIsBrowser] = useState(false);
  const [isSupported, setIsSupported] = useState(true);
  const [isLoading, setIsLoading] = useState(true);
  const { settings, updateSettings, addRecordedVideo } = useCameraContext();

  // Check if we're in the browser
  useEffect(() => {
    setIsBrowser(true);

    // Check if the browser supports the camera API
    if (typeof window !== "undefined") {
      if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        setIsSupported(false);
        setError(
          "Camera API is not supported in your browser. Please try using Chrome, Firefox, or Safari.",
        );
        setIsLoading(false);
      }
    }
  }, []);

  // Handle video element events
  useEffect(() => {
    if (!videoRef.current || !isBrowser) return;

    const videoElement = videoRef.current;

    const handleCanPlay = () => {
      console.log("Video can play now");
      setIsLoading(false);
    };

    const handleError = (e: Event) => {
      console.error("Video element error:", e);
      setError(
        "Error displaying camera feed. Please check permissions and refresh.",
      );
      setIsLoading(false);
    };

    videoElement.addEventListener("canplay", handleCanPlay);
    videoElement.addEventListener("error", handleError);

    return () => {
      videoElement.removeEventListener("canplay", handleCanPlay);
      videoElement.removeEventListener("error", handleError);
    };
  }, [isBrowser, videoRef.current]);

  // Initialize camera once when component mounts
  useEffect(() => {
    // Only run this effect in the browser and if the API is supported
    if (!isBrowser || !isSupported) return;

    let mounted = true;

    async function initializeCamera() {
      // If we already have a stream, no need to initialize again
      if (stream) return;

      try {
        setIsLoading(true);

        // Basic initialization with default settings
        const constraints = {
          audio: settings.recordAudio,
          video: {
            facingMode: settings.cameraFacing,
          },
        };

        console.log(
          "Initial camera initialization with constraints:",
          constraints,
        );

        const mediaStream =
          await navigator.mediaDevices.getUserMedia(constraints);

        // Only set state if component is still mounted
        if (mounted) {
          setStream(mediaStream);
          setIsLoading(false);
        } else {
          // Clean up stream if component unmounted during initialization
          mediaStream.getTracks().forEach((track) => track.stop());
        }
      } catch (err) {
        console.error("Error initializing camera:", err);
        if (mounted) {
          setError(
            `Could not access camera: ${err instanceof Error ? err.message : String(err)}. Please check permissions and make sure you're using HTTPS.`,
          );
          setIsLoading(false);
        }
      }
    }

    initializeCamera();

    // Cleanup function
    return () => {
      mounted = false;
    };
  }, [isBrowser, isSupported]); // Only depend on browser/support status

  // Update camera settings
  useEffect(() => {
    // Only run this effect in the browser and if the API is supported and we have an initial stream
    if (!isBrowser || !isSupported || !stream) return;

    let setupAttempts = 0;
    const maxAttempts = 5;

    async function setupCamera() {
      try {
        setIsLoading(true);
        console.log("Setting up camera with settings:", settings);

        // Stop any existing stream
        stream.getTracks().forEach((track) => {
          console.log("Stopping track:", track.kind, track.label);
          track.stop();
        });

        // Get quality settings
        let width, height;
        switch (settings.videoQuality) {
          case "low":
            width = 640;
            height = 480;
            break;
          case "medium":
            width = 1280;
            height = 720;
            break;
          case "high":
            width = 1920;
            height = 1080;
            break;
          case "ultra":
            width = 3840;
            height = 2160;
            break;
          default:
            width = 1280;
            height = 720;
        }

        console.log("Requesting camera with constraints:", {
          audio: settings.recordAudio,
          video: {
            width: { ideal: width },
            height: { ideal: height },
            facingMode: settings.cameraFacing,
          },
        });

        const constraints = {
          audio: settings.recordAudio,
          video: {
            width: { ideal: width },
            height: { ideal: height },
            facingMode: settings.cameraFacing,
          },
        };

        const mediaStream =
          await navigator.mediaDevices.getUserMedia(constraints);
        console.log("Got media stream:", mediaStream);
        console.log(
          "Video tracks:",
          mediaStream
            .getVideoTracks()
            .map((t) => ({ label: t.label, enabled: t.enabled })),
        );

        setStream(mediaStream);

        if (videoRef.current) {
          console.log("Setting video source");
          videoRef.current.srcObject = mediaStream;

          // Ensure the video plays
          try {
            await videoRef.current.play();
            console.log("Video playback started");
          } catch (playError) {
            console.error("Error playing video:", playError);
          }
        } else {
          console.error("Video ref is not available");

          // Retry setup if video ref is not available and we haven't reached max attempts
          setupAttempts++;
          if (setupAttempts < maxAttempts) {
            console.log(
              `Retrying camera setup (attempt ${setupAttempts}/${maxAttempts})...`,
            );
            setTimeout(setupCamera, 500); // Wait 500ms before trying again
            return;
          } else {
            setError(
              "Could not initialize video element after multiple attempts. Please refresh the page.",
            );
            setIsLoading(false);
          }
        }
      } catch (err) {
        console.error("Error accessing camera:", err);
        setError(
          `Could not access camera: ${err instanceof Error ? err.message : String(err)}. Please check permissions and make sure you're using HTTPS.`,
        );
        setIsLoading(false);
      }
    }

    setupCamera();

    return () => {
      // No need to stop the stream here as we will handle it in the setupCamera function
    };
  }, [
    isBrowser,
    isSupported,
    stream,
    settings.videoQuality,
    settings.cameraFacing,
    settings.recordAudio,
  ]);

  // Cleanup when component unmounts
  useEffect(() => {
    return () => {
      if (stream) {
        stream.getTracks().forEach((track) => track.stop());
      }
    };
  }, [stream]);

  const handleRecordingComplete = (blob: Blob) => {
    const url = URL.createObjectURL(blob);

    // Add to our context
    addRecordedVideo(url);

    console.log("Recording complete, video URL:", url);
  };

  const toggleRecording = () => {
    setIsRecording(!isRecording);
  };

  const switchCamera = async () => {
    if (!stream) return;

    try {
      // Get current facing mode
      const videoTrack = stream.getVideoTracks()[0];
      if (!videoTrack) return;

      const currentFacingMode = videoTrack.getSettings().facingMode;
      const newFacingMode =
        currentFacingMode === "user" ? "environment" : "user";

      console.log(
        "Switching camera from",
        currentFacingMode,
        "to",
        newFacingMode,
      );

      // Update context with new facing mode
      updateSettings({ cameraFacing: newFacingMode });
    } catch (err) {
      console.error("Error switching camera:", err);
    }
  };

  const requestCameraPermission = async () => {
    try {
      setIsLoading(true);
      setError(null);

      // Request minimal video access to trigger the permission prompt
      const stream = await navigator.mediaDevices.getUserMedia({ video: true });

      // Stop the stream immediately as we'll request it again with proper settings
      stream.getTracks().forEach((track) => track.stop());

      // Refresh the camera with proper settings
      const { videoQuality, cameraFacing, recordAudio } = settings;
      updateSettings({ videoQuality, cameraFacing, recordAudio });
    } catch (err) {
      console.error("Error requesting camera permission:", err);
      setError(
        `Camera permission denied: ${err instanceof Error ? err.message : String(err)}`,
      );
      setIsLoading(false);
    }
  };

  if (!isSupported) {
    return (
      <div className="flex h-full flex-col items-center justify-center p-4 text-center">
        <AlertCircle className="mb-4 h-12 w-12 text-yellow-500" />
        <h2 className="mb-2 text-xl font-bold">Browser Not Supported</h2>
        <p className="mb-6 text-zinc-400">
          Your browser doesn't support the camera API. Please try using Chrome,
          Firefox, or Safari.
        </p>
        <p className="text-sm text-zinc-500">
          If you're using a supported browser, make sure you're accessing this
          site via HTTPS.
        </p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex h-full flex-col items-center justify-center p-4 text-center">
        <AlertCircle className="mb-4 h-12 w-12 text-red-500" />
        <p className="mb-4 text-red-500">{error}</p>
        <button
          className="rounded-full bg-zinc-800 px-6 py-3 text-white hover:bg-zinc-700"
          onClick={requestCameraPermission}
        >
          Grant Camera Access
        </button>
      </div>
    );
  }

  // Show loading state while checking browser compatibility or loading camera
  if (!isBrowser || isLoading) {
    return (
      <div className="flex h-full flex-col items-center justify-center">
        <div className="flex animate-pulse flex-col items-center">
          <CameraIcon className="mb-4 h-12 w-12 text-zinc-600" />
          <p className="mb-4 text-white">Initializing camera...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="relative flex h-full flex-col items-center justify-center">
      {/* Video Recorder Component (no UI) */}
      {stream && (
        <VideoRecorder
          stream={stream}
          isRecording={isRecording}
          onRecordingComplete={handleRecordingComplete}
        />
      )}

      {/* Camera Preview */}
      <div className="relative mx-auto aspect-[9/16] w-full max-w-3xl overflow-hidden rounded-lg bg-zinc-900">
        <video
          ref={videoRef}
          autoPlay
          playsInline
          muted
          className="h-full w-full object-cover"
          onLoadedMetadata={() => {
            console.log("Video metadata loaded");
            if (videoRef.current) {
              videoRef.current.play().catch((err) => {
                console.error(
                  "Error playing video after metadata loaded:",
                  err,
                );
              });
            }
          }}
        />

        {/* Recording Indicator */}
        {isRecording && (
          <div className="absolute right-4 top-4 flex items-center">
            <div className="mr-2 h-3 w-3 animate-pulse rounded-full bg-red-600"></div>
            <span className="text-sm font-medium text-white">REC</span>
          </div>
        )}
      </div>

      {/* Controls */}
      <div className="absolute bottom-24 left-0 right-0 flex items-center justify-center gap-6">
        <button
          onClick={switchCamera}
          className="rounded-full bg-zinc-800 p-3"
          aria-label="Switch camera"
        >
          <RefreshCw className="h-6 w-6" />
        </button>

        <button
          onClick={toggleRecording}
          className={`rounded-full p-5 ${
            isRecording
              ? "bg-red-600 hover:bg-red-700"
              : "bg-red-600 hover:bg-red-700"
          }`}
          aria-label={isRecording ? "Stop recording" : "Start recording"}
        >
          {isRecording ? (
            <Square className="h-6 w-6" />
          ) : (
            <Play className="h-6 w-6" />
          )}
        </button>
      </div>
    </div>
  );
}
