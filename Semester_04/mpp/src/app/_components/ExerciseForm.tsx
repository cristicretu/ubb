"use client";

import { useState, useEffect } from "react";
import { useCameraContext, Exercise } from "./CameraContext";

interface ExerciseFormProps {
  videoUrl: string;
  duration: number;
  onCancel: () => void;
  onSave: () => void;
  initialData?: Partial<Exercise>;
}

export default function ExerciseForm({
  videoUrl,
  duration,
  onCancel,
  onSave,
  initialData,
}: ExerciseFormProps) {
  const { addExercise, updateExercise, exercises } = useCameraContext();
  const [name, setName] = useState(initialData?.name || "");
  const [form, setForm] = useState<"bad" | "medium" | "good">(
    initialData?.form || "medium",
  );

  const handleSave = () => {
    if (!name) return;

    console.log("Saving exercise with name:", name);

    if (initialData?.id) {
      console.log("Updating existing exercise:", initialData.id);
      updateExercise(initialData.id, {
        name,
        form,
      });
    } else {
      console.log("Adding new exercise with video:", videoUrl);
      addExercise({
        name,
        videoUrl,
        form,
        date: new Date().toISOString(),
        duration,
      });
    }

    onSave();
  };

  return (
    <div className="rounded-lg border border-gray-300 p-4">
      <h2 className="mb-4 text-xl font-bold">
        {initialData?.id ? "Edit Exercise" : "Save Exercise"}
      </h2>

      {videoUrl && (
        <div className="mb-4">
          <video
            src={videoUrl}
            controls
            className="h-48 w-full rounded-md object-cover"
            playsInline
          />
        </div>
      )}

      <div className="mb-4">
        <label className="mb-2 block font-medium">Exercise Name</label>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          className="w-full rounded-md border border-gray-300 p-2 text-black"
          placeholder="Enter exercise name"
        />
      </div>

      <div className="mb-4">
        <label className="mb-2 block font-medium">Form Quality</label>
        <div className="flex gap-4">
          <button
            onClick={() => setForm("bad")}
            className={`rounded-md px-4 py-2 ${
              form === "bad"
                ? "bg-red-600 text-white"
                : "bg-gray-200 text-black"
            }`}
          >
            Bad
          </button>
          <button
            onClick={() => setForm("medium")}
            className={`rounded-md px-4 py-2 ${
              form === "medium"
                ? "bg-yellow-500 text-white"
                : "bg-gray-200 text-black"
            }`}
          >
            Medium
          </button>
          <button
            onClick={() => setForm("good")}
            className={`rounded-md px-4 py-2 ${
              form === "good"
                ? "bg-green-600 text-white"
                : "bg-gray-200 text-black"
            }`}
          >
            Good
          </button>
        </div>
      </div>

      <div className="flex justify-end gap-2">
        <button
          onClick={onCancel}
          className="rounded-md bg-gray-500 px-4 py-2 text-white hover:bg-gray-600"
        >
          Cancel
        </button>
        <button
          onClick={handleSave}
          disabled={!name}
          className={`rounded-md bg-blue-600 px-4 py-2 text-white hover:bg-blue-700 ${
            !name && "cursor-not-allowed opacity-50"
          }`}
        >
          {initialData?.id ? "Update" : "Save"} Exercise
        </button>
      </div>
    </div>
  );
}
