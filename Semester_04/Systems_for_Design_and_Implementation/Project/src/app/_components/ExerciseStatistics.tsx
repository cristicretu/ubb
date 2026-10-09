"use client";

import { useState } from "react";
import { Exercise } from "./CameraContext";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";

interface ExerciseStatisticsProps {
  exercises: Exercise[];
}

export default function ExerciseStatistics({
  exercises,
}: ExerciseStatisticsProps) {
  // Return early if no exercises
  if (!exercises.length) {
    return (
      <Card className="bg-card border-border shadow-sm">
        <CardHeader className="pb-2">
          <CardTitle className="text-lg font-medium">
            Exercise Statistics
          </CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-muted-foreground text-center">
            No exercises to analyze
          </p>
        </CardContent>
      </Card>
    );
  }

  const durations = exercises.map((ex) => ex.duration);
  const minDuration = Math.min(...durations);
  const maxDuration = Math.max(...durations);
  const avgDuration = Math.round(
    durations.reduce((acc, dur) => acc + dur, 0) / durations.length,
  );

  const formCounts = {
    bad: exercises.filter((ex) => ex.form === "bad").length,
    medium: exercises.filter((ex) => ex.form === "medium").length,
    good: exercises.filter((ex) => ex.form === "good").length,
  };

  const totalExercises = exercises.length;
  const formPercentages = {
    bad: Math.round((formCounts.bad / totalExercises) * 100),
    medium: Math.round((formCounts.medium / totalExercises) * 100),
    good: Math.round((formCounts.good / totalExercises) * 100),
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  };

  const minDurationEx = exercises.find((ex) => ex.duration === minDuration);
  const maxDurationEx = exercises.find((ex) => ex.duration === maxDuration);

  const avgDurationEx = [...exercises].sort(
    (a, b) =>
      Math.abs(a.duration - avgDuration) - Math.abs(b.duration - avgDuration),
  )[0];

  return (
    <Card className="border-border mb-6 border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">
          Exercise Statistics
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          <StatCard
            title="Duration Stats"
            items={[
              {
                label: "Shortest",
                value: formatDuration(minDuration),
                id: minDurationEx?.id,
                color: "green",
              },
              {
                label: "Average",
                value: formatDuration(avgDuration),
                id: avgDurationEx?.id,
                color: "blue",
              },
              {
                label: "Longest",
                value: formatDuration(maxDuration),
                id: maxDurationEx?.id,
                color: "red",
              },
            ]}
          />

          <StatCard
            title="Form Quality"
            items={[
              {
                label: "Bad",
                value: `${formCounts.bad} (${formPercentages.bad}%)`,
                color: "red",
              },
              {
                label: "Medium",
                value: `${formCounts.medium} (${formPercentages.medium}%)`,
                color: "yellow",
              },
              {
                label: "Good",
                value: `${formCounts.good} (${formPercentages.good}%)`,
                color: "green",
              },
            ]}
          />

          <StatCard
            title="Overall"
            items={[
              { label: "Total Exercises", value: totalExercises.toString() },
              {
                label: "Total Duration",
                value: formatDuration(durations.reduce((a, b) => a + b, 0)),
              },
              {
                label: "Recent Activity",
                value: exercises.length > 0 ? "Active" : "None",
                color: exercises.length > 0 ? "green" : "red",
              },
            ]}
          />
        </div>

        <div className="mt-6">
          <h3 className="mb-2 font-medium">Form Distribution</h3>
          <div className="flex h-6 w-full overflow-hidden rounded-full bg-slate-200 shadow-inner">
            {formCounts.bad > 0 && (
              <div
                className="h-full bg-red-500"
                style={{ width: `${formPercentages.bad}%` }}
                title={`Bad: ${formCounts.bad} (${formPercentages.bad}%)`}
              />
            )}
            {formCounts.medium > 0 && (
              <div
                className="h-full bg-yellow-500"
                style={{ width: `${formPercentages.medium}%` }}
                title={`Medium: ${formCounts.medium} (${formPercentages.medium}%)`}
              />
            )}
            {formCounts.good > 0 && (
              <div
                className="h-full bg-green-500"
                style={{ width: `${formPercentages.good}%` }}
                title={`Good: ${formCounts.good} (${formPercentages.good}%)`}
              />
            )}
          </div>
          <div className="mt-2 flex justify-between text-xs text-gray-500">
            <span>Bad ({formPercentages.bad}%)</span>
            <span>Medium ({formPercentages.medium}%)</span>
            <span>Good ({formPercentages.good}%)</span>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}

function StatCard({
  title,
  items,
}: {
  title: string;
  items: Array<{
    label: string;
    value: string;
    color?: string;
    id?: string;
  }>;
}) {
  return (
    <div className="border-border bg-card/50 hover:bg-card/80 rounded-lg border p-4 shadow-sm transition-colors">
      <h3 className="mb-3 font-medium">{title}</h3>
      <div className="space-y-2.5">
        {items.map((item, i) => (
          <div key={i} className="flex items-center justify-between">
            <span className="text-muted-foreground text-sm">{item.label}</span>
            <span className="font-medium" data-exercise-id={item.id || ""}>
              {item.color ? (
                <Badge variant={item.color as any} className="ml-2">
                  {item.value}
                </Badge>
              ) : (
                item.value
              )}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}
