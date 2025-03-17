"use client";

import { useEffect, useState } from "react";
import PageLayout from "../_components/PageLayout";
import { useCameraContext, Exercise } from "../_components/CameraContext";
import ExerciseForm from "../_components/ExerciseForm";
import Link from "next/link";

type SortOption = "date-newest" | "date-oldest" | "name-asc" | "name-desc";

export default function Gallery() {
  const { exercises, deleteExercise } = useCameraContext();
  const [formFilter, setFormFilter] = useState<
    "all" | "bad" | "medium" | "good"
  >("all");
  const [sortBy, setSortBy] = useState<SortOption>("date-newest");
  const [searchTerm, setSearchTerm] = useState("");
  const [editingExercise, setEditingExercise] = useState<Exercise | null>(null);

  // Debug output
  useEffect(() => {
    console.log("Gallery rendering with exercises:", exercises);
  }, [exercises]);

  const formatDate = (dateStr: string): string => {
    const date = new Date(dateStr);
    return new Intl.DateTimeFormat("en-US", {
      month: "short",
      day: "2-digit",
      year: "numeric",
      hour: "2-digit",
      minute: "2-digit",
      hour12: true,
    }).format(date);
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const filteredExercises = exercises.filter((exercise) => {
    const matchesForm = formFilter === "all" || exercise.form === formFilter;
    const matchesSearch = exercise.name
      .toLowerCase()
      .includes(searchTerm.toLowerCase());
    return matchesForm && matchesSearch;
  });

  const sortedExercises = [...filteredExercises].sort((a, b) => {
    switch (sortBy) {
      case "date-newest":
        return new Date(b.date).getTime() - new Date(a.date).getTime();
      case "date-oldest":
        return new Date(a.date).getTime() - new Date(b.date).getTime();
      case "name-asc":
        return a.name.localeCompare(b.name);
      case "name-desc":
        return b.name.localeCompare(a.name);
      default:
        return 0;
    }
  });

  const getFormBadgeClass = (form: "bad" | "medium" | "good") => {
    switch (form) {
      case "bad":
        return "bg-red-600";
      case "medium":
        return "bg-yellow-500";
      case "good":
        return "bg-green-600";
      default:
        return "bg-gray-500";
    }
  };

  return (
    <PageLayout>
      <div className="container mx-auto px-4 py-8">
        <div className="mb-6 flex items-center justify-between">
          <h1 className="text-3xl font-bold">Exercise Gallery</h1>
          <div className="flex items-center gap-2">
            <span className="text-sm text-zinc-400">
              {exercises.length} exercises
            </span>
            <Link
              href="/"
              className="rounded-md bg-zinc-800 px-4 py-2 text-sm text-white hover:bg-zinc-700"
            >
              Back to Camera
            </Link>
          </div>
        </div>

        <div className="mb-6 rounded-lg bg-zinc-800 p-4">
          <h2 className="mb-3 font-semibold">Filter & Sort</h2>
          <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
            <div>
              <label className="mb-1 block text-sm">Search</label>
              <input
                type="text"
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                placeholder="Search by name..."
                className="w-full rounded-md border border-zinc-600 bg-zinc-700 px-3 py-2 text-sm"
              />
            </div>
            <div>
              <label className="mb-1 block text-sm">Form Quality</label>
              <select
                value={formFilter}
                onChange={(e) => setFormFilter(e.target.value as any)}
                className="w-full rounded-md border border-zinc-600 bg-zinc-700 px-3 py-2 text-sm"
              >
                <option value="all">All Forms</option>
                <option value="bad">Bad</option>
                <option value="medium">Medium</option>
                <option value="good">Good</option>
              </select>
            </div>
            <div>
              <label className="mb-1 block text-sm">Sort By</label>
              <select
                value={sortBy}
                onChange={(e) => setSortBy(e.target.value as SortOption)}
                className="w-full rounded-md border border-zinc-600 bg-zinc-700 px-3 py-2 text-sm"
              >
                <option value="date-newest">Date (Newest First)</option>
                <option value="date-oldest">Date (Oldest First)</option>
                <option value="name-asc">Name (A-Z)</option>
                <option value="name-desc">Name (Z-A)</option>
              </select>
            </div>
          </div>
        </div>

        {editingExercise && (
          <div className="mb-6">
            <ExerciseForm
              videoUrl={editingExercise.videoUrl}
              duration={editingExercise.duration}
              initialData={editingExercise}
              onCancel={() => setEditingExercise(null)}
              onSave={() => setEditingExercise(null)}
            />
          </div>
        )}

        {sortedExercises.length > 0 ? (
          <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
            {sortedExercises.map((exercise) => (
              <div
                key={exercise.id}
                className="overflow-hidden rounded-lg bg-zinc-900"
              >
                <video
                  src={exercise.videoUrl}
                  controls
                  className="w-full rounded-t-lg"
                  playsInline
                />
                <div className="p-4">
                  <div className="mb-2 flex items-center justify-between">
                    <h3 className="font-semibold">{exercise.name}</h3>
                    <span
                      className={`rounded-full ${getFormBadgeClass(exercise.form)} px-2 py-1 text-xs text-white`}
                    >
                      {exercise.form.charAt(0).toUpperCase() +
                        exercise.form.slice(1)}
                    </span>
                  </div>
                  <p className="mb-1 text-sm text-zinc-400">
                    {formatDate(exercise.date)}
                  </p>
                  <p className="mb-3 text-sm text-zinc-400">
                    Duration: {formatDuration(exercise.duration)}
                  </p>
                  <div className="flex gap-2">
                    <button
                      onClick={() => setEditingExercise(exercise)}
                      className="flex-1 rounded-md bg-blue-600 px-4 py-2 text-xs font-medium text-white hover:bg-blue-700"
                    >
                      Edit
                    </button>
                    <button
                      onClick={() => deleteExercise(exercise.id)}
                      className="flex-1 rounded-md bg-red-600 px-4 py-2 text-xs font-medium text-white hover:bg-red-700"
                    >
                      Delete
                    </button>
                  </div>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <div className="flex flex-col items-center justify-center py-20">
            <div className="max-w-md rounded-lg bg-zinc-900 p-8 text-center">
              <h2 className="mb-2 text-xl font-semibold">No exercises found</h2>
              <p className="mb-4 text-zinc-400">
                {exercises.length > 0
                  ? "Try adjusting your filters to see more exercises."
                  : "Your recorded exercises will appear here. Go to the Camera page to start recording."}
              </p>
              <Link
                href="/"
                className="inline-block rounded-md bg-blue-600 px-4 py-2 text-sm font-medium text-white hover:bg-blue-700"
              >
                Go to Camera
              </Link>
            </div>
          </div>
        )}
      </div>
    </PageLayout>
  );
}
