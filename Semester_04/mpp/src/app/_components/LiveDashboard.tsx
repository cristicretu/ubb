"use client";

import { useState, useEffect, useCallback } from "react";
import { Exercise, useCameraContext } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Activity, TrendingUp, Clock, Zap, Award } from "lucide-react";

export default function LiveDashboard() {
  const { exercises, addEventListener, removeEventListener } =
    useCameraContext();
  const [recentActivity, setRecentActivity] = useState<{
    type: "add" | "update" | "delete";
    timestamp: number;
    exerciseName?: string;
  } | null>(null);

  const [showActivity, setShowActivity] = useState(false);
  const [stats, setStats] = useState({
    todayCount: 0,
    todayDuration: 0,
    streak: 0,
    recentExercises: [] as Exercise[],
  });

  const calculateStats = useCallback(() => {
    if (!exercises.length) return;

    const today = new Date();
    today.setHours(0, 0, 0, 0);

    const todayExercises = exercises.filter((ex) => {
      const exDate = new Date(ex.date);
      return exDate >= today;
    });

    const todayDuration = todayExercises.reduce(
      (sum, ex) => sum + ex.duration,
      0,
    );

    let streak = 0;
    const exerciseDays = new Set<string>();

    exercises.forEach((ex) => {
      const date = new Date(ex.date);
      exerciseDays.add(
        `${date.getFullYear()}-${date.getMonth()}-${date.getDate()}`,
      );
    });

    const checkDate = new Date(today);
    while (
      exerciseDays.has(
        `${checkDate.getFullYear()}-${checkDate.getMonth()}-${checkDate.getDate()}`,
      )
    ) {
      streak++;
      checkDate.setDate(checkDate.getDate() - 1);
    }

    const recentExercises = [...exercises]
      .sort((a, b) => new Date(b.date).getTime() - new Date(a.date).getTime())
      .slice(0, 5);

    setStats({
      todayCount: todayExercises.length,
      todayDuration,
      streak,
      recentExercises,
    });
  }, [exercises]);

  useEffect(() => {
    const handleExercisesChange = () => {
      calculateStats();

      setRecentActivity({
        type: "update",
        timestamp: Date.now(),
        exerciseName: "Exercise",
      });

      setShowActivity(true);

      setTimeout(() => {
        setShowActivity(false);
      }, 5000);
    };

    calculateStats();

    addEventListener("exercisesChange", handleExercisesChange);

    return () => {
      removeEventListener("exercisesChange", handleExercisesChange);
    };
  }, [addEventListener, removeEventListener, calculateStats]);

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, "0")}`;
  };

  const formatRelativeTime = (dateStr: string): string => {
    const date = new Date(dateStr);
    const now = new Date();
    const diffMs = now.getTime() - date.getTime();

    const diffSec = Math.floor(diffMs / 1000);

    if (diffSec < 60) return `${diffSec} seconds ago`;
    if (diffSec < 3600) return `${Math.floor(diffSec / 60)} minutes ago`;
    if (diffSec < 86400) return `${Math.floor(diffSec / 3600)} hours ago`;
    return `${Math.floor(diffSec / 86400)} days ago`;
  };

  return (
    <>
      <div
        className={`fixed right-4 top-4 z-50 transform transition-all duration-500 ${
          showActivity
            ? "translate-y-0 opacity-100"
            : "-translate-y-8 opacity-0"
        }`}
        style={{ maxWidth: "300px" }}
      >
        <div className="rounded-lg bg-green-500 px-4 py-3 text-white shadow-lg">
          <div className="flex items-center gap-2">
            <Activity className="h-4 w-4 animate-pulse" />
            <span>Exercise data updated</span>
          </div>
        </div>
      </div>

      {/* Dashboard cards */}
      <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-4">
        {/* Today's exercises */}
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-base font-medium">
              Today's Exercises
            </CardTitle>
            <CardDescription>Exercise count for today</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex items-center">
              <Zap className="mr-2 h-5 w-5 text-blue-500" />
              <span className="text-2xl font-bold">{stats.todayCount}</span>
            </div>
            <p className="text-muted-foreground mt-2 text-sm">
              {stats.todayCount > 0
                ? `${formatDuration(stats.todayDuration)} total duration`
                : "No exercises recorded today"}
            </p>
          </CardContent>
        </Card>

        {/* Current streak */}
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-base font-medium">
              Current Streak
            </CardTitle>
            <CardDescription>Consecutive days with exercises</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex items-center">
              <TrendingUp className="mr-2 h-5 w-5 text-orange-500" />
              <span className="text-2xl font-bold">{stats.streak}</span>
              <span className="text-muted-foreground ml-1">days</span>
            </div>
            <p className="text-muted-foreground mt-2 text-sm">
              {stats.streak > 0
                ? `Keep going! You're building a habit.`
                : "Start your streak today!"}
            </p>
          </CardContent>
        </Card>

        {/* Exercise count */}
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-base font-medium">
              Total Exercises
            </CardTitle>
            <CardDescription>All recorded exercises</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex items-center">
              <Activity className="mr-2 h-5 w-5 text-purple-500" />
              <span className="text-2xl font-bold">{exercises.length}</span>
            </div>
            <p className="text-muted-foreground mt-2 text-sm">
              {exercises.length > 0
                ? `Great progress! Keep it up.`
                : "Record your first exercise to get started."}
            </p>
          </CardContent>
        </Card>

        {/* Top form quality */}
        <Card>
          <CardHeader className="pb-2">
            <CardTitle className="text-base font-medium">
              Form Quality
            </CardTitle>
            <CardDescription>Exercise form breakdown</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="flex items-center gap-3">
              <Award className="h-5 w-5 text-green-500" />
              <div className="flex gap-2">
                <Badge variant="outline" className="text-green-500">
                  {exercises.filter((ex) => ex.form === "good").length} Good
                </Badge>
                <Badge variant="outline" className="text-yellow-500">
                  {exercises.filter((ex) => ex.form === "medium").length} Medium
                </Badge>
                <Badge variant="outline" className="text-red-500">
                  {exercises.filter((ex) => ex.form === "bad").length} Bad
                </Badge>
              </div>
            </div>
            <div className="mt-3 h-2 w-full overflow-hidden rounded-full bg-gray-200">
              {exercises.length > 0 && (
                <div className="flex h-full">
                  <div
                    className="h-full bg-green-500"
                    style={{
                      width: `${(exercises.filter((ex) => ex.form === "good").length / exercises.length) * 100}%`,
                    }}
                  />
                  <div
                    className="h-full bg-yellow-500"
                    style={{
                      width: `${(exercises.filter((ex) => ex.form === "medium").length / exercises.length) * 100}%`,
                    }}
                  />
                  <div
                    className="h-full bg-red-500"
                    style={{
                      width: `${(exercises.filter((ex) => ex.form === "bad").length / exercises.length) * 100}%`,
                    }}
                  />
                </div>
              )}
            </div>
          </CardContent>
        </Card>
      </div>

      {stats.recentExercises.length > 0 && (
        <Card className="mt-6">
          <CardHeader className="pb-2">
            <CardTitle className="text-lg font-medium">
              Recent Activity
            </CardTitle>
            <CardDescription>Your latest exercises</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              {stats.recentExercises.map((exercise) => (
                <div
                  key={exercise.id}
                  className="flex items-center justify-between"
                >
                  <div className="flex items-center gap-3">
                    <div
                      className={`h-2 w-2 rounded-full ${
                        exercise.form === "good"
                          ? "bg-green-500"
                          : exercise.form === "medium"
                            ? "bg-yellow-500"
                            : "bg-red-500"
                      }`}
                    />
                    <span className="font-medium">{exercise.name}</span>
                    <Badge variant="outline" className="text-xs">
                      {formatDuration(exercise.duration)}
                    </Badge>
                  </div>
                  <span className="text-muted-foreground text-xs">
                    {formatRelativeTime(exercise.date)}
                  </span>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>
      )}
    </>
  );
}
