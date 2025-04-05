"use client";

import { useState } from "react";
import FileUpload from "./FileUpload";
import VideoPreview from "./VideoPreview";
import { useCameraContext } from "./CameraContext";
import ExerciseForm from "./ExerciseForm";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
} from "~/components/ui/dialog";
import { toast } from "sonner";

export default function UploadVideo() {
  const { addRecordedVideo } = useCameraContext();
  const [uploadedVideoUrl, setUploadedVideoUrl] = useState<string | null>(null);
  const [showExerciseForm, setShowExerciseForm] = useState(false);
  const [videoDuration, setVideoDuration] = useState(0);

  const handleUploadComplete = (fileUrl: string) => {
    setUploadedVideoUrl(fileUrl);
    addRecordedVideo(fileUrl);
    setShowExerciseForm(true);
  };

  const handleLoadedMetadata = (duration: number) => {
    setVideoDuration(Math.round(duration));
  };

  return (
    <div className="flex w-full max-w-md flex-col space-y-4">
      <div className="flex flex-col space-y-2">
        <h3 className="text-lg font-semibold">Upload Exercise Video</h3>
        <p className="text-sm text-gray-400">
          Upload a video of your exercise to analyze your form.
        </p>
      </div>

      <FileUpload
        onUploadComplete={handleUploadComplete}
        accept="video/*"
        className="mb-4"
      />

      {uploadedVideoUrl && (
        <div className="mt-4">
          <h4 className="text-md mb-2 font-medium">Preview</h4>
          <VideoPreview
            src={uploadedVideoUrl}
            height="240px"
            controls
            onLoadedMetadata={handleLoadedMetadata}
          />
        </div>
      )}

      <Dialog open={showExerciseForm} onOpenChange={setShowExerciseForm}>
        <DialogContent className="sm:max-w-md">
          <DialogHeader>
            <DialogTitle>Save Exercise</DialogTitle>
            <DialogDescription>
              Review and save your uploaded exercise.
            </DialogDescription>
          </DialogHeader>
          {uploadedVideoUrl && (
            <ExerciseForm
              videoUrl={uploadedVideoUrl}
              duration={videoDuration}
              onCancel={() => setShowExerciseForm(false)}
              onSave={() => {
                setShowExerciseForm(false);
                toast.success("Exercise saved successfully");
                setUploadedVideoUrl(null);
              }}
            />
          )}
        </DialogContent>
      </Dialog>
    </div>
  );
}
