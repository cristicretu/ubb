"use client";

import { useState, useEffect } from "react";
import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

interface ExerciseProgressChartProps {
  exercises: Exercise[];
}

export default function ExerciseProgressChart({
  exercises,
}: ExerciseProgressChartProps) {
  const [averageIntensityByDay, setAverageIntensityByDay] = useState<
    {
      date: string;
      displayDate: string;
      intensity: number;
      count: number;
      form: {
        good: number;
        medium: number;
        bad: number;
      };
    }[]
  >([]);

  useEffect(() => {
    const calculateIntensity = () => {
      if (!exercises.length) return [];

      const exercisesByDay: {
        [key: string]: {
          exercises: Exercise[];
          totalDuration: number;
        };
      } = {};

      exercises.forEach((exercise) => {
        const date = new Date(exercise.date);
        const dateKey = date.toISOString().split("T")[0];

        if (!exercisesByDay[dateKey]) {
          exercisesByDay[dateKey] = {
            exercises: [],
            totalDuration: 0,
          };
        }

        exercisesByDay[dateKey].exercises.push(exercise);
        exercisesByDay[dateKey].totalDuration += exercise.duration;
      });

      const intensityData = Object.entries(exercisesByDay).map(
        ([dateKey, data]) => {
          const { exercises, totalDuration } = data;
          const count = exercises.length;

          const formCounts = {
            good: exercises.filter((ex) => ex.form === "good").length,
            medium: exercises.filter((ex) => ex.form === "medium").length,
            bad: exercises.filter((ex) => ex.form === "bad").length,
          };

          const qualityFactor =
            (formCounts.good * 1.0 +
              formCounts.medium * 0.7 +
              formCounts.bad * 0.4) /
            count;

          const intensity = (totalDuration * qualityFactor) / count;

          const date = new Date(dateKey);
          const displayDate = date.toLocaleDateString("en-US", {
            month: "short",
            day: "numeric",
          });

          return {
            date: dateKey,
            displayDate,
            intensity,
            count,
            form: formCounts,
          };
        },
      );

      return intensityData.sort((a, b) => a.date.localeCompare(b.date));
    };

    const intensityData = calculateIntensity();
    setAverageIntensityByDay(intensityData);
  }, [exercises]);

  if (!exercises.length) {
    return null;
  }

  const chartHeight = 180;
  const maxIntensity = Math.max(
    ...averageIntensityByDay.map((day) => day.intensity),
    1,
  );

  const points = averageIntensityByDay.map((day, index) => {
    const x = (index / (averageIntensityByDay.length - 1 || 1)) * 100;
    const y = 100 - (day.intensity / maxIntensity) * 100;
    return `${x},${y}`;
  });

  const areaPoints = [`0,100`, ...points, `100,100`].join(" ");

  const linePoints = points.join(" ");

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">
          Exercise Intensity
        </CardTitle>
        <CardDescription>
          Workout intensity over time based on duration and form quality
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="relative h-[220px] w-full pt-4">
          <div className="absolute left-0 top-0 flex h-full w-10 flex-col justify-between py-2 text-xs text-gray-500">
            <span>High</span>
            <span>Med</span>
            <span>Low</span>
          </div>

          <div className="absolute inset-y-0 left-10 right-0">
            <div className="absolute inset-0 flex flex-col justify-between">
              <div className="border-t border-gray-200" />
              <div className="border-t border-gray-200" />
              <div className="border-t border-gray-200" />
            </div>

            <div className="relative h-full w-full">
              <svg
                viewBox="0 0 100 100"
                preserveAspectRatio="none"
                className="absolute inset-0 h-full w-full overflow-visible"
              >
                <defs>
                  <linearGradient
                    id="intensityGradient"
                    x1="0"
                    y1="0"
                    x2="0"
                    y2="1"
                  >
                    <stop
                      offset="0%"
                      stopColor="rgb(79, 70, 229)"
                      stopOpacity="0.7"
                    />
                    <stop
                      offset="100%"
                      stopColor="rgb(79, 70, 229)"
                      stopOpacity="0.1"
                    />
                  </linearGradient>
                </defs>
                <polygon points={areaPoints} fill="url(#intensityGradient)" />
                {/* Line */}
                <polyline
                  points={linePoints}
                  fill="none"
                  stroke="rgb(79, 70, 229)"
                  strokeWidth="0.8"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
                {/* Data points */}
                {points.map((point, index) => {
                  const [x, y] = point.split(",").map(parseFloat);
                  return (
                    <circle
                      key={index}
                      cx={x}
                      cy={y}
                      r="1.5"
                      fill="white"
                      stroke="rgb(79, 70, 229)"
                      strokeWidth="0.5"
                      className="hover:r-2 transition-all"
                    >
                      <title>
                        {averageIntensityByDay[index].displayDate}:
                        {Math.round(averageIntensityByDay[index].intensity)}{" "}
                        intensity ({averageIntensityByDay[index].count}{" "}
                        exercises)
                      </title>
                    </circle>
                  );
                })}
              </svg>
            </div>

            <div className="absolute bottom-0 left-0 right-0 flex justify-between px-2 pt-2 text-xs text-gray-500">
              {averageIntensityByDay.length > 7
                ? averageIntensityByDay
                    .filter(
                      (_, i) =>
                        i % Math.ceil(averageIntensityByDay.length / 5) === 0 ||
                        i === averageIntensityByDay.length - 1,
                    )
                    .map((day, i) => <span key={i}>{day.displayDate}</span>)
                : averageIntensityByDay.map((day, i) => (
                    <span key={i}>{day.displayDate}</span>
                  ))}
            </div>
          </div>
        </div>

        <div className="mt-4 flex items-center justify-center gap-6">
          <div className="flex items-center gap-2">
            <div className="h-3 w-3 rounded-full bg-indigo-600 opacity-70"></div>
            <span className="text-xs text-gray-500">Intensity</span>
          </div>
          <div className="text-xs text-gray-500">
            <span className="font-medium">Latest:</span>{" "}
            {averageIntensityByDay.length > 0
              ? `${Math.round(averageIntensityByDay[averageIntensityByDay.length - 1].intensity)} units`
              : "N/A"}
          </div>
          <div className="text-xs text-gray-500">
            <span className="font-medium">Highest:</span>{" "}
            {averageIntensityByDay.length > 0
              ? `${Math.round(Math.max(...averageIntensityByDay.map((d) => d.intensity)))} units`
              : "N/A"}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
