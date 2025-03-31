"use client";

import { useState, useEffect } from "react";
import { useCameraContext, Exercise } from "./CameraContext";
import { toast } from "sonner";

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
  const [nameError, setNameError] = useState("");
  const [form, setForm] = useState<"bad" | "medium" | "good">(
    initialData?.form || "medium",
  );
  const [isSaving, setIsSaving] = useState(false);

  const validateName = (value: string) => {
    if (!value.trim()) {
      setNameError("Exercise name cannot be empty");
      return false;
    }

    if (/\d/.test(value)) {
      setNameError("Exercise name cannot contain numbers");
      return false;
    }

    setNameError("");
    return true;
  };

  const handleNameChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setName(e.target.value);
    if (nameError) setNameError("");
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") {
      e.preventDefault();
      handleSave();
    }
  };

  const handleSave = async () => {
    if (!validateName(name)) return;

    setIsSaving(true);

    try {
      if (initialData?.id) {
        await updateExercise(initialData.id, {
          name,
          form,
        });
        toast.success(`Exercise "${name}" updated successfully`);
      } else {
        await addExercise({
          name,
          videoUrl,
          form,
          date: new Date().toISOString(),
          duration,
        });
        toast.success(`Exercise "${name}" saved successfully`);
      }

      onSave();
    } catch (error) {
      console.error("Error saving exercise:", error);
      let errorMessage = "Failed to save exercise. Please try again.";
      if (error instanceof Error) {
        errorMessage = error.message || errorMessage;
      }
      toast.error(errorMessage);
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <div className="space-y-4">
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
          onChange={handleNameChange}
          onKeyDown={handleKeyDown}
          onBlur={() => validateName(name)}
          className="w-full rounded-md border border-gray-300 p-2 text-black"
          placeholder="Enter exercise name"
        />
        {nameError && <p className="mt-1 text-sm text-red-500">{nameError}</p>}
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
          disabled={isSaving}
        >
          Cancel
        </button>
        <button
          onClick={handleSave}
          disabled={!name || isSaving}
          className={`rounded-md bg-blue-600 px-4 py-2 text-white hover:bg-blue-700 ${
            (!name || isSaving) && "cursor-not-allowed opacity-50"
          }`}
        >
          {isSaving ? (
            <span className="flex items-center gap-2">
              <svg className="h-4 w-4 animate-spin" viewBox="0 0 24 24">
                <circle
                  className="opacity-25"
                  cx="12"
                  cy="12"
                  r="10"
                  stroke="currentColor"
                  strokeWidth="4"
                  fill="none"
                />
                <path
                  className="opacity-75"
                  fill="currentColor"
                  d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                />
              </svg>
              {initialData?.id ? "Updating" : "Saving"}...
            </span>
          ) : (
            `${initialData?.id ? "Update" : "Save"} Exercise`
          )}
        </button>
      </div>
    </div>
  );
}
