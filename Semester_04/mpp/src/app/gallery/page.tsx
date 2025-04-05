"use client";

import { useEffect, useState, useCallback, useMemo, useRef } from "react";
import PageLayout from "../_components/PageLayout";
import {
  useCameraContext,
  Exercise,
  ExerciseFilters,
} from "../_components/CameraContext";
import ExerciseForm from "../_components/ExerciseForm";
import ExerciseStatistics from "../_components/ExerciseStatistics";
import ExerciseDurationChart from "../_components/ExerciseDurationChart";
import ExerciseQualityChart from "../_components/ExerciseQualityChart";
import WeeklyProgressChart from "../_components/WeeklyProgressChart";
import ExerciseProgressChart from "../_components/ExerciseProgressChart";
import LiveDashboard from "../_components/LiveDashboard";
import InfiniteScroll from "../_components/InfiniteScroll";
import { useNetwork } from "../_components/NetworkContext";
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
import { Toaster } from "~/components/ui/sonner";

type SortOption =
  | "date-newest"
  | "date-oldest"
  | "name-asc"
  | "name-desc"
  | "duration-highest"
  | "duration-lowest";

export default function Gallery() {
  const { isOnline, isServerAvailable } = useNetwork();
  const {
    exercises,
    deleteExercise,
    fetchFilteredExercises,
    hasMoreExercises,
    isLoadingMore,
    loadMoreExercises,
    addEventListener,
    removeEventListener,
    isSyncing,
    pendingOperationsCount,
    syncPendingOperations,
  } = useCameraContext();

  const fetchExercisesRef = useRef(fetchFilteredExercises);
  const fetchStatsTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    fetchExercisesRef.current = fetchFilteredExercises;
  }, [fetchFilteredExercises]);

  console.log("Raw exercises from context:", exercises);

  const [formFilter, setFormFilter] = useState<
    "all" | "bad" | "medium" | "good"
  >("all");
  const [sortBy, setSortBy] = useState<SortOption>("date-newest");
  const [searchTerm, setSearchTerm] = useState("");
  const [editingExercise, setEditingExercise] = useState<Exercise | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);

  const [filteredExercises, setFilteredExercises] = useState<Exercise[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [totalExercises, setTotalExercises] = useState(0);

  const [allExercisesForStats, setAllExercisesForStats] = useState<Exercise[]>(
    [],
  );
  const [isLoadingStats, setIsLoadingStats] = useState(false);

  const [chartKey, setChartKey] = useState(0);

  const exercisesArray =
    allExercisesForStats.length > 0
      ? allExercisesForStats
      : Array.isArray(exercises)
        ? exercises
        : [];

  console.log("Exercises array length:", exercisesArray.length);

  const durations = exercisesArray.map((ex) => ex.duration);
  const minDuration = durations.length > 0 ? Math.min(...durations) : 0;
  const maxDuration = durations.length > 0 ? Math.max(...durations) : 0;
  const avgDuration =
    durations.length > 0
      ? durations.reduce((acc, dur) => acc + dur, 0) / durations.length
      : 0;

  const minDurationExId = exercisesArray.find(
    (ex) => ex.duration === minDuration,
  )?.id;
  const maxDurationExId = exercisesArray.find(
    (ex) => ex.duration === maxDuration,
  )?.id;

  const avgDurationExId =
    exercisesArray.length > 0
      ? [...exercisesArray].sort(
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

  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  useEffect(() => {
    const handleExerciseChange = async () => {
      const filters: ExerciseFilters = {
        form: formFilter === "all" ? undefined : formFilter,
        search: searchTerm || undefined,
        sortBy:
          sortBy === "date-newest"
            ? "date-desc"
            : sortBy === "date-oldest"
              ? "date-asc"
              : sortBy === "name-asc"
                ? "name-asc"
                : sortBy === "name-desc"
                  ? "name-desc"
                  : sortBy === "duration-highest"
                    ? "duration-desc"
                    : sortBy === "duration-lowest"
                      ? "duration-asc"
                      : "date-desc",
        page: 1,
        limit: 12,
      };

      try {
        setIsLoading(true);
        const result = await fetchExercisesRef.current(filters);
        setFilteredExercises(result.exercises);
        setTotalExercises(result.total);
      } catch (error) {
        console.error("Failed to refresh exercises after changes:", error);
      } finally {
        setIsLoading(false);
      }
    };

    addEventListener("exercisesChange", handleExerciseChange);

    return () => {
      removeEventListener("exercisesChange", handleExerciseChange);
    };
  }, [addEventListener, removeEventListener, formFilter, searchTerm, sortBy]);

  useEffect(() => {
    if (
      isClient &&
      (isOnline !== undefined || isServerAvailable !== undefined) &&
      !isSyncing
    ) {
      const loadFilteredExercises = async () => {
        setIsLoading(true);
        try {
          const filters: ExerciseFilters = {
            form: formFilter === "all" ? undefined : formFilter,
            search: searchTerm || undefined,
            sortBy:
              sortBy === "date-newest"
                ? "date-desc"
                : sortBy === "date-oldest"
                  ? "date-asc"
                  : sortBy === "name-asc"
                    ? "name-asc"
                    : sortBy === "name-desc"
                      ? "name-desc"
                      : sortBy === "duration-highest"
                        ? "duration-desc"
                        : sortBy === "duration-lowest"
                          ? "duration-asc"
                          : "date-desc",
            page: 1,
            limit: 12,
          };

          const result = await fetchExercisesRef.current(filters);
          setFilteredExercises(result.exercises);
          setTotalExercises(result.total);
        } catch (error) {
          console.error(
            "Failed to load filtered exercises on network change:",
            error,
          );
        } finally {
          setIsLoading(false);
        }
      };

      loadFilteredExercises();
    }
  }, [
    isClient,
    isOnline,
    isServerAvailable,
    formFilter,
    searchTerm,
    sortBy,
    isSyncing,
  ]);

  useEffect(() => {
    const loadFilteredExercises = async () => {
      if (isSyncing) return;

      setIsLoading(true);
      try {
        const filters: ExerciseFilters = {
          form: formFilter === "all" ? undefined : formFilter,
          search: searchTerm || undefined,
          sortBy:
            sortBy === "date-newest"
              ? "date-desc"
              : sortBy === "date-oldest"
                ? "date-asc"
                : sortBy === "name-asc"
                  ? "name-asc"
                  : sortBy === "name-desc"
                    ? "name-desc"
                    : sortBy === "duration-highest"
                      ? "duration-desc"
                      : sortBy === "duration-lowest"
                        ? "duration-asc"
                        : "date-desc",
          page: 1,
          limit: 12,
        };

        const result = await fetchExercisesRef.current(filters);
        console.log("Filtered exercises result:", result);
        setFilteredExercises(result.exercises);
        setTotalExercises(result.total);
      } catch (error) {
        console.error("Failed to load filtered exercises:", error);
      } finally {
        setIsLoading(false);
      }
    };

    loadFilteredExercises();
  }, [formFilter, searchTerm, sortBy, isSyncing]);

  useEffect(() => {
    if (isClient && !isSyncing && isOnline && isServerAvailable) {
      const loadFilteredExercises = async () => {
        setIsLoading(true);
        try {
          const filters: ExerciseFilters = {
            form: formFilter === "all" ? undefined : formFilter,
            search: searchTerm || undefined,
            sortBy:
              sortBy === "date-newest"
                ? "date-desc"
                : sortBy === "date-oldest"
                  ? "date-asc"
                  : sortBy === "name-asc"
                    ? "name-asc"
                    : sortBy === "name-desc"
                      ? "name-desc"
                      : sortBy === "duration-highest"
                        ? "duration-desc"
                        : sortBy === "duration-lowest"
                          ? "duration-asc"
                          : "date-desc",
            page: 1,
            limit: 12,
          };

          const result = await fetchExercisesRef.current(filters);
          setFilteredExercises(result.exercises);
          setTotalExercises(result.total);
        } catch (error) {
          console.error("Failed to refresh exercises after sync:", error);
        } finally {
          setIsLoading(false);
        }
      };

      loadFilteredExercises();
    }
  }, [
    isSyncing,
    isClient,
    isOnline,
    isServerAvailable,
    formFilter,
    searchTerm,
    sortBy,
  ]);

  const handleLoadMore = async () => {
    try {
      console.log(
        "Infinite scroll triggered, hasMoreExercises:",
        hasMoreExercises,
      );
      console.log(
        "Current filtered exercises count:",
        filteredExercises.length,
      );

      const currentLoadedPage = Math.ceil(filteredExercises.length / 12);
      const nextPage = currentLoadedPage + 1;

      console.log("Loading page:", nextPage);

      const filters: ExerciseFilters = {
        form: formFilter === "all" ? undefined : formFilter,
        search: searchTerm || undefined,
        sortBy:
          sortBy === "date-newest"
            ? "date-desc"
            : sortBy === "date-oldest"
              ? "date-asc"
              : sortBy === "name-asc"
                ? "name-asc"
                : sortBy === "name-desc"
                  ? "name-desc"
                  : sortBy === "duration-highest"
                    ? "duration-desc"
                    : sortBy === "duration-lowest"
                      ? "duration-asc"
                      : "date-desc",
        page: nextPage,
        limit: 12,
      };

      console.log("Fetching next page with filters:", filters);

      const result = await fetchExercisesRef.current(filters);
      console.log("Next page result:", result);

      if (result.exercises && result.exercises.length > 0) {
        setFilteredExercises((prevExercises) => [
          ...prevExercises,
          ...result.exercises,
        ]);

        const hasMore =
          result.exercises.length === 12 &&
          nextPage < Math.ceil(result.total / 12);
        console.log(
          "Has more pages:",
          hasMore,
          "Current page:",
          nextPage,
          "Total pages:",
          Math.ceil(result.total / 12),
        );

        fetchAllExercisesForStats();
        return true;
      } else {
        console.log("No more exercises to load");
        return false;
      }
    } catch (error) {
      console.error("Error in handleLoadMore:", error);
      return false;
    }
  };

  useEffect(() => {
    setChartKey((prevKey) => prevKey + 1);
  }, [exercises]);

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

  const fetchAllExercisesForStats = useCallback(async () => {
    if (isSyncing) return;

    if (fetchStatsTimeoutRef.current) {
      clearTimeout(fetchStatsTimeoutRef.current);
      fetchStatsTimeoutRef.current = null;
    }

    fetchStatsTimeoutRef.current = setTimeout(async () => {
      setIsLoadingStats(true);
      try {
        const statsFilters: ExerciseFilters = {
          form: formFilter === "all" ? undefined : formFilter,
          search: searchTerm || undefined,
          sortBy:
            sortBy === "date-newest"
              ? "date-desc"
              : sortBy === "date-oldest"
                ? "date-asc"
                : sortBy === "name-asc"
                  ? "name-asc"
                  : sortBy === "name-desc"
                    ? "name-desc"
                    : sortBy === "duration-highest"
                      ? "duration-desc"
                      : sortBy === "duration-lowest"
                        ? "duration-asc"
                        : "date-desc",
          page: 1,
          limit: 1000,
        };

        const result = await fetchExercisesRef.current(statsFilters);
        console.log("Stats fetch result:", result);
        setAllExercisesForStats(result.exercises);
        setTotalExercises(result.total);

        setChartKey((prev) => prev + 1);
      } catch (error) {
        console.error("Failed to load all exercises for statistics:", error);
      } finally {
        setIsLoadingStats(false);
        fetchStatsTimeoutRef.current = null;
      }
    }, 300);
  }, [formFilter, searchTerm, sortBy, isSyncing]);

  useEffect(() => {
    if (
      isClient &&
      (isOnline !== undefined || isServerAvailable !== undefined) &&
      !isSyncing
    ) {
      fetchAllExercisesForStats();
    }
  }, [
    isClient,
    isOnline,
    isServerAvailable,
    isSyncing,
    fetchAllExercisesForStats,
  ]);

  useEffect(() => {
    const handleExerciseChange = () => {
      fetchAllExercisesForStats();
    };

    addEventListener("exercisesChange", handleExerciseChange);

    return () => {
      removeEventListener("exercisesChange", handleExerciseChange);
    };
  }, [addEventListener, removeEventListener, fetchAllExercisesForStats]);

  useEffect(() => {
    return () => {
      if (fetchStatsTimeoutRef.current) {
        clearTimeout(fetchStatsTimeoutRef.current);
      }
    };
  }, []);

  return (
    <PageLayout>
      <Toaster />
      <div
        style={{ height: "calc(100vh - 64px)", overflowY: "auto" }}
        className="container mx-auto px-4 py-8"
      >
        <div className="mb-8 flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Button variant="ghost" size="icon" asChild>
              <Link href="/">
                <ArrowLeft className="h-5 w-5" />
              </Link>
            </Button>
            <h1 className="text-3xl font-bold tracking-tight">Exercises</h1>
            {isSyncing && (
              <Badge
                variant="outline"
                className="ml-2 bg-yellow-100 text-yellow-800"
              >
                <div className="mr-1 h-2 w-2 animate-pulse rounded-full bg-yellow-500"></div>
                Syncing...
              </Badge>
            )}
          </div>
          <div className="flex items-center gap-2">
            <Badge variant="secondary" className="text-sm">
              {totalExercises} exercise
              {totalExercises !== 1 ? "s" : ""}
            </Badge>
            {pendingOperationsCount > 0 &&
              isOnline &&
              isServerAvailable &&
              !isSyncing && (
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => {
                    syncPendingOperations();
                  }}
                >
                  Sync ({pendingOperationsCount})
                </Button>
              )}
            <Button asChild>
              <Link href="/">Record New</Link>
            </Button>
          </div>
        </div>

        {isClient && exercisesArray.length > -1 && (
          <>
            <div className="mb-8">
              <LiveDashboard
                exercisesData={allExercisesForStats}
                totalCount={totalExercises}
              />
            </div>

            {isLoadingStats ? (
              <div className="flex justify-center py-4">
                <div className="flex items-center space-x-2">
                  <div className="border-primary h-5 w-5 animate-spin rounded-full border-2 border-t-transparent"></div>
                  <span>Loading statistics...</span>
                </div>
              </div>
            ) : (
              <>
                <ExerciseStatistics
                  key={`stats-${chartKey}`}
                  exercises={exercisesArray}
                />

                <div className="mb-8 grid grid-cols-1 gap-6 md:grid-cols-2">
                  {exercisesArray.length > 1 && (
                    <ExerciseDurationChart
                      key={`duration-${chartKey}`}
                      exercises={exercisesArray}
                    />
                  )}
                  <ExerciseQualityChart
                    key={`quality-${chartKey}`}
                    exercises={exercisesArray}
                  />
                </div>

                <div className="mb-8 grid grid-cols-1 gap-6 md:grid-cols-2">
                  <WeeklyProgressChart
                    key={`weekly-${chartKey}`}
                    exercises={exercisesArray}
                  />
                  <ExerciseProgressChart
                    key={`progress-${chartKey}`}
                    exercises={exercisesArray}
                  />
                </div>
              </>
            )}
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
                    <SelectValue placeholder="Sort By" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="date-newest">
                      Date (Newest First)
                    </SelectItem>
                    <SelectItem value="date-oldest">
                      Date (Oldest First)
                    </SelectItem>
                    <SelectItem value="name-asc">Name (A-Z)</SelectItem>
                    <SelectItem value="name-desc">Name (Z-A)</SelectItem>
                    <SelectItem value="duration-highest">
                      Duration (Highest)
                    </SelectItem>
                    <SelectItem value="duration-lowest">
                      Duration (Lowest)
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
                  setChartKey((prevKey) => prevKey + 1);

                  setTimeout(async () => {
                    try {
                      const filters: ExerciseFilters = {
                        form: formFilter === "all" ? undefined : formFilter,
                        search: searchTerm || undefined,
                        sortBy:
                          sortBy === "date-newest"
                            ? "date-desc"
                            : sortBy === "date-oldest"
                              ? "date-asc"
                              : sortBy === "name-asc"
                                ? "name-asc"
                                : sortBy === "name-desc"
                                  ? "name-desc"
                                  : sortBy === "duration-highest"
                                    ? "duration-desc"
                                    : sortBy === "duration-lowest"
                                      ? "duration-asc"
                                      : "date-desc",
                        page: 1,
                        limit: 12,
                      };

                      const result = await fetchExercisesRef.current(filters);
                      setFilteredExercises(result.exercises);
                    } catch (error) {
                      console.error(
                        "Failed to refresh exercises after edit:",
                        error,
                      );
                    }
                  }, 100);
                }}
              />
            )}
          </DialogContent>
        </Dialog>

        {filteredExercises.length > 0 ? (
          <InfiniteScroll
            loadMore={handleLoadMore}
            hasMore={hasMoreExercises}
            isLoading={isLoadingMore}
            loadingComponent={
              <div className="col-span-full flex justify-center py-4">
                <div className="flex items-center space-x-2">
                  <div className="border-primary h-5 w-5 animate-spin rounded-full border-2 border-t-transparent"></div>
                  <span>Loading more exercises...</span>
                </div>
              </div>
            }
            endComponent={
              <div className="col-span-full flex justify-center py-4">
                <span className="text-muted-foreground">
                  No more exercises to load
                </span>
              </div>
            }
          >
            <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 md:grid-cols-2 lg:grid-cols-3">
              {isLoading ? (
                <div className="col-span-full flex justify-center py-8">
                  <div className="animate-pulse text-center">Loading...</div>
                </div>
              ) : (
                filteredExercises.map((exercise) => {
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
                          <CardTitle className="text-lg">
                            {exercise.name}
                          </CardTitle>
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
                          <span>
                            Duration: {formatDuration(exercise.duration)}
                          </span>

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
                            onClick={() => {
                              deleteExercise(exercise.id);
                              setFilteredExercises((prev) =>
                                prev.filter((ex) => ex.id !== exercise.id),
                              );
                              setChartKey((prevKey) => prevKey + 1);
                            }}
                            className="flex-1"
                          >
                            Delete
                          </Button>
                        </div>
                      </CardFooter>
                    </Card>
                  );
                })
              )}
            </div>
          </InfiniteScroll>
        ) : (
          <div className="mt-16 flex flex-col items-center justify-center">
            <div className="bg-muted mb-4 flex h-16 w-16 items-center justify-center rounded-full">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="text-muted-foreground h-8 w-8"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z"
                />
              </svg>
            </div>
            <h3 className="mb-2 text-xl font-semibold">No exercises found</h3>
            <p className="text-muted-foreground mb-6 text-center">
              {searchTerm || formFilter !== "all"
                ? "Try adjusting your filters"
                : !isClient
                  ? "Start by recording a new exercise"
                  : !isOnline || !isServerAvailable
                    ? "You are offline. Record new exercises or reconnect to see your data."
                    : "Start by recording a new exercise"}
            </p>
            <Button asChild>
              <Link href="/">Record New Exercise</Link>
            </Button>
          </div>
        )}
      </div>
    </PageLayout>
  );
}
