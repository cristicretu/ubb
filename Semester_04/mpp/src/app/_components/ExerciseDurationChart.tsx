"use client";

import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

interface ExerciseDurationChartProps {
  exercises: Exercise[];
}

export default function ExerciseDurationChart({
  exercises,
}: ExerciseDurationChartProps) {
  if (!exercises.length) {
    return null;
  }

  const sortedExercises = [...exercises].sort(
    (a, b) => new Date(a.date).getTime() - new Date(b.date).getTime(),
  );

  const durations = sortedExercises.map((ex) => ex.duration);

  const maxDuration = Math.max(...durations);

  const chartHeight = 160;
  const barWidth = 100 / (durations.length > 1 ? durations.length - 1 : 1);
  const barGap = Math.min(4, barWidth / 4);
  const adjustedBarWidth = barWidth - barGap;

  const avgDuration =
    durations.reduce((sum, val) => sum + val, 0) / durations.length;

  const formatDate = (dateString: string) => {
    const date = new Date(dateString);
    return new Intl.DateTimeFormat("en-US", {
      month: "short",
      day: "numeric",
    }).format(date);
  };

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, "0")}`;
  };

  return (
    <Card className="mb-6">
      <CardHeader className="pb-2">
        <CardTitle>Duration Trend</CardTitle>
        <CardDescription>
          Exercise durations over time. Average: {formatDuration(avgDuration)}
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="relative h-[200px] w-full">
          {/* Chart Y-axis labels */}
          <div className="absolute left-0 top-0 flex h-full w-12 flex-col justify-between px-2 text-xs text-gray-500">
            <span>{formatDuration(maxDuration)}</span>
            <span>{formatDuration(maxDuration / 2)}</span>
            <span>0:00</span>
          </div>
          <div className="absolute inset-y-0 left-12 right-0">
            <div
              className="absolute left-0 right-0 border-t border-dashed border-blue-400"
              style={{
                top: `${(1 - avgDuration / maxDuration) * chartHeight}px`,
              }}
            />

            <div className="absolute inset-0 border-b border-l border-gray-200">
              <div className="absolute inset-0 flex flex-col justify-between">
                <div className="border-t border-gray-200" />
                <div className="border-t border-gray-200" />
                <div className="border-t border-gray-200" />
              </div>
            </div>

            <div className="absolute inset-0 flex items-end">
              {sortedExercises.map((exercise, index) => {
                const height = (exercise.duration / maxDuration) * chartHeight;
                const formColor = {
                  bad: "bg-red-500",
                  medium: "bg-yellow-500",
                  good: "bg-green-500",
                }[exercise.form];

                return (
                  <div
                    key={exercise.id}
                    className="flex flex-col items-center"
                    style={{ width: `${barWidth}%` }}
                  >
                    <div
                      className={`mb-1 rounded-t opacity-90 transition-all hover:opacity-100 ${formColor}`}
                      style={{
                        height: `${height}px`,
                        width: `${adjustedBarWidth}%`,
                      }}
                      title={`${exercise.name}: ${formatDuration(exercise.duration)}`}
                    />
                    <span className="mt-1 w-full rotate-45 truncate text-xs text-gray-500">
                      {formatDate(exercise.date)}
                    </span>
                  </div>
                );
              })}
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
