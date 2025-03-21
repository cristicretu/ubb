"use client";

import { useEffect, useState } from "react";
import PageLayout from "../_components/PageLayout";
import { useCameraContext, Exercise } from "../_components/CameraContext";
import ExerciseForm from "../_components/ExerciseForm";
import ExerciseStatistics from "../_components/ExerciseStatistics";
import ExerciseDurationChart from "../_components/ExerciseDurationChart";
import ExerciseQualityChart from "../_components/ExerciseQualityChart";
import Link from "next/link";
import { Search, Filter, Calendar, ArrowUpDown, ArrowLeft } from "lucide-react";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "~/components/ui/dialog";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import {
  Card,
  CardContent,
  CardFooter,
  CardHeader,
  CardTitle,
  CardDescription,
} from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";

type SortOption =
  | "date-newest"
  | "date-oldest"
  | "name-asc"
  | "name-desc"
  | "duration-highest"
  | "duration-lowest";

export default function Gallery() {
  const { exercises, deleteExercise } = useCameraContext();
  const [formFilter, setFormFilter] = useState<
    "all" | "bad" | "medium" | "good"
  >("all");
  const [sortBy, setSortBy] = useState<SortOption>("date-newest");
  const [searchTerm, setSearchTerm] = useState("");
  const [editingExercise, setEditingExercise] = useState<Exercise | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);

  const durations = exercises.map((ex) => ex.duration);
  const minDuration = Math.min(...durations);
  const maxDuration = Math.max(...durations);
  const avgDuration =
    durations.length > 0
      ? durations.reduce((acc, dur) => acc + dur, 0) / durations.length
      : 0;

  const minDurationExId = exercises.find(
    (ex) => ex.duration === minDuration,
  )?.id;
  const maxDurationExId = exercises.find(
    (ex) => ex.duration === maxDuration,
  )?.id;

  const avgDurationExId =
    exercises.length > 0
      ? [...exercises].sort(
          (a, b) =>
            Math.abs(a.duration - avgDuration) -
            Math.abs(b.duration - avgDuration),
        )[0]?.id || null
      : null;

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
      case "duration-highest":
        return b.duration - a.duration;
      case "duration-lowest":
        return a.duration - b.duration;
      default:
        return 0;
    }
  });

  const getExerciseHighlight = (exerciseId: string) => {
    if (exerciseId === maxDurationExId) {
      return {
        borderColor: "border-red-500",
        label: "Longest Duration",
      };
    } else if (exerciseId === minDurationExId) {
      return {
        borderColor: "border-green-500",
        label: "Shortest Duration",
      };
    } else if (exerciseId === avgDurationExId) {
      return {
        borderColor: "border-blue-500",
        label: "Average Duration",
      };
    }
    return { borderColor: "", label: "" };
  };

  return (
    <PageLayout>
      <div className="container mx-auto max-w-7xl px-4 py-8">
        <div className="mb-8 flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Button variant="ghost" size="icon" asChild>
              <Link href="/">
                <ArrowLeft className="h-5 w-5" />
              </Link>
            </Button>
            <h1 className="text-3xl font-bold tracking-tight">Exercises</h1>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant="secondary" className="text-sm">
              {exercises.length} exercise{exercises.length !== 1 ? "s" : ""}
            </Badge>
            <Button asChild>
              <Link href="/">Record New</Link>
            </Button>
          </div>
        </div>

        {exercises.length > 0 && (
          <>
            <ExerciseStatistics exercises={exercises} />

            <div className="mb-8 grid grid-cols-1 gap-6 md:grid-cols-2">
              {exercises.length > 1 && (
                <ExerciseDurationChart exercises={exercises} />
              )}
              <ExerciseQualityChart exercises={exercises} />
            </div>
          </>
        )}

        <div className="mb-8 space-y-4">
          <div className="flex flex-col gap-4 sm:flex-row">
            <div className="relative flex-1">
              <Search className="absolute left-3 top-1/2 h-4 w-4 -translate-y-1/2 text-zinc-500" />
              <Input
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                placeholder="Search exercises..."
                className="pl-9"
              />
            </div>
            <div className="flex flex-col gap-4 sm:flex-row">
              <div className="w-full sm:w-48">
                <Select
                  value={formFilter}
                  onValueChange={(value) => setFormFilter(value as any)}
                >
                  <SelectTrigger>
                    <SelectValue placeholder="Form Quality" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="all">All Forms</SelectItem>
                    <SelectItem value="bad">Bad</SelectItem>
                    <SelectItem value="medium">Medium</SelectItem>
                    <SelectItem value="good">Good</SelectItem>
                  </SelectContent>
                </Select>
              </div>
              <div className="w-full sm:w-48">
                <Select
                  value={sortBy}
                  onValueChange={(value) => setSortBy(value as SortOption)}
                >
                  <SelectTrigger>
                    <SelectValue placeholder="Sort by" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="date-newest">Newest First</SelectItem>
                    <SelectItem value="date-oldest">Oldest First</SelectItem>
                    <SelectItem value="name-asc">Name (A-Z)</SelectItem>
                    <SelectItem value="name-desc">Name (Z-A)</SelectItem>
                    <SelectItem value="duration-highest">
                      Longest Duration
                    </SelectItem>
                    <SelectItem value="duration-lowest">
                      Shortest Duration
                    </SelectItem>
                  </SelectContent>
                </Select>
              </div>
            </div>
          </div>
        </div>

        <Dialog
          open={dialogOpen && editingExercise !== null}
          onOpenChange={(open) => {
            setDialogOpen(open);
            if (!open) setEditingExercise(null);
          }}
        >
          <DialogContent className="sm:max-w-md">
            <DialogHeader>
              <DialogTitle>Edit Exercise</DialogTitle>
              <DialogDescription>
                Make changes to your exercise recording.
              </DialogDescription>
            </DialogHeader>
            {editingExercise && (
              <ExerciseForm
                videoUrl={editingExercise.videoUrl}
                duration={editingExercise.duration}
                initialData={editingExercise}
                onCancel={() => {
                  setDialogOpen(false);
                  setEditingExercise(null);
                }}
                onSave={() => {
                  setDialogOpen(false);
                  setEditingExercise(null);
                }}
              />
            )}
          </DialogContent>
        </Dialog>

        {sortedExercises.length > 0 ? (
          <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
            {sortedExercises.map((exercise) => {
              const highlight = getExerciseHighlight(exercise.id);

              return (
                <Card
                  key={exercise.id}
                  className={`flex flex-col overflow-hidden shadow-sm transition-all ${
                    highlight.borderColor
                      ? `border-2 ${highlight.borderColor}`
                      : "border-border border"
                  }`}
                >
                  <div className="aspect-video overflow-hidden">
                    <video
                      src={exercise.videoUrl}
                      controls
                      className="h-full w-full object-cover"
                      playsInline
                    />
                  </div>
                  <CardHeader className="pb-2">
                    <div className="flex items-center justify-between">
                      <CardTitle className="text-xl">{exercise.name}</CardTitle>
                      <Badge variant={exercise.form as any}>
                        {exercise.form.charAt(0).toUpperCase() +
                          exercise.form.slice(1)}
                      </Badge>
                    </div>
                    <CardDescription className="flex items-center gap-2">
                      <Calendar className="h-3.5 w-3.5" />
                      {formatDate(exercise.date)}
                    </CardDescription>
                    <div className="mt-2 flex items-center justify-between text-sm text-zinc-500">
                      <span>Duration: {formatDuration(exercise.duration)}</span>

                      {highlight.label && (
                        <Badge
                          variant="outline"
                          className={`${highlight.borderColor.replace("border-", "text-")}`}
                        >
                          {highlight.label}
                        </Badge>
                      )}
                    </div>
                  </CardHeader>
                  <CardFooter className="mt-auto pt-0">
                    <div className="flex w-full gap-2">
                      <Button
                        variant="outline"
                        onClick={() => {
                          setEditingExercise(exercise);
                          setDialogOpen(true);
                        }}
                        className="flex-1"
                      >
                        Edit
                      </Button>
                      <Button
                        variant="destructive"
                        onClick={() => deleteExercise(exercise.id)}
                        className="flex-1"
                      >
                        Delete
                      </Button>
                    </div>
                  </CardFooter>
                </Card>
              );
            })}
          </div>
        ) : (
          <Card className="border-border mx-auto w-full max-w-md border shadow-sm">
            <CardHeader>
              <CardTitle className="text-center">No exercises found</CardTitle>
              <CardDescription className="text-center">
                {exercises.length > 0
                  ? "Try adjusting your filters to see more exercises."
                  : "Your recorded exercises will appear here."}
              </CardDescription>
            </CardHeader>
            <CardContent className="text-center">
              <p className="text-muted-foreground mb-4">
                Record your first exercise to get started.
              </p>
              <Button asChild>
                <Link href="/">Record New Exercise</Link>
              </Button>
            </CardContent>
          </Card>
        )}
      </div>
    </PageLayout>
  );
}
